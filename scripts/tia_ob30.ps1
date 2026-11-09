<#
  Stage 2 of the TIA build: add FC_LoadParams, the DBs and OB30 (cyclic interrupt, 100 ms)
  from plc\tia\FC_LoadParams.scl and plc\tia\LineIO_OB_DB.scl to a project made by
  tia_build.ps1, then compile. (Adapted from the author's tcu-zone-pid-tuning project.)

  TIA Openness V18 has no CreateOB, and an OB generated from an SCL source becomes a
  Program-cycle OB.  So: generate from source, export the OB as SimaticML, switch it to
  SecondaryType CyclicInterrupt / number 30 / CyclicTime 100000 us, delete and re-import.

  Usage: powershell -ExecutionPolicy Bypass -File scripts\tia_ob30.ps1 -Project <path.ap18>
#>
param(
    [Parameter(Mandatory = $true)][string]$Project,
    [string]$Version = "V18"
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$api = "C:\Program Files\Siemens\Automation\Portal $Version\PublicAPI\$Version\Siemens.Engineering.dll"
$portal = "C:\Program Files\Siemens\Automation\Portal $Version"

$resolver = @"
using System; using System.IO; using System.Reflection;
public static class OpnResolve2 {
    static string[] dirs;
    public static void Init(string[] d) {
        dirs = d;
        AppDomain.CurrentDomain.AssemblyResolve += (s, e) => {
            var n = new AssemblyName(e.Name).Name;
            if (n.EndsWith(".resources")) return null;
            foreach (var dir in dirs) { var p = Path.Combine(dir, n + ".dll"); if (File.Exists(p)) return Assembly.LoadFrom(p); }
            return null;
        };
    }
}
"@
Add-Type -TypeDefinition $resolver
[OpnResolve2]::Init(@((Split-Path -Parent $api), "$portal\Bin\PublicAPI", "$portal\Bin"))
[void][System.Reflection.Assembly]::LoadFrom($api)

$code = @"
using System;
using System.IO;
using System.Text;
using System.Xml;
using Siemens.Engineering;
using Siemens.Engineering.HW;
using Siemens.Engineering.HW.Features;
using Siemens.Engineering.SW;
using Siemens.Engineering.SW.Blocks;
using Siemens.Engineering.Compiler;

public static class XtrOb30 {
    static void Walk(CompilerResultMessage m, StringBuilder sb, int depth) {
        sb.AppendLine(new string(' ', 2 * depth) + m.State + "  " + m.Path + "  " + m.Description);
        foreach (var c in m.Messages) Walk(c, sb, depth + 1);
    }
    static void Set(XmlDocument d, XmlNode attrs, string name, string value) {
        XmlNode n = null;
        foreach (XmlNode c in attrs.ChildNodes) if (c.LocalName == name) n = c;
        if (n == null) { n = d.CreateElement(name, attrs.NamespaceURI); attrs.AppendChild(n); }
        n.InnerText = value;
    }
    public static string Run(string projectPath, string sourcePath, string workDir) {
        var sb = new StringBuilder();
        using (var tia = new TiaPortal(TiaPortalMode.WithoutUserInterface)) {
            var project = tia.Projects.Open(new FileInfo(projectPath));
            PlcSoftware plc = null;
            foreach (var dev in project.Devices)
                foreach (var item in dev.DeviceItems) {
                    var sc = item.GetService<SoftwareContainer>();
                    if (sc != null && sc.Software is PlcSoftware) plc = (PlcSoftware)sc.Software;
                }
            // sourcePath: ';'-separated list, generated in order (the FC before the OB that calls it)
            foreach (var sp in sourcePath.Split(';')) {
                var nm = Path.GetFileNameWithoutExtension(sp);
                var src = plc.ExternalSourceGroup.ExternalSources.CreateFromFile(nm, sp);
                try { src.GenerateBlocksFromSource(); sb.AppendLine("generated " + nm); }
                catch (Exception ex) { sb.AppendLine("GENERATE FAILED " + nm + ": " + (ex.InnerException ?? ex).Message); }
            }

            var first = plc.GetService<ICompilable>().Compile();  // export needs consistent blocks
            sb.AppendLine("compile before export: " + first.State + " errors=" + first.ErrorCount + " warnings=" + first.WarningCount);
            foreach (var m in first.Messages) Walk(m, sb, 1);
            var ob = plc.BlockGroup.Blocks.Find("Cyclic_Pressure");
            var xmlIn = new FileInfo(Path.Combine(workDir, "Cyclic_Pressure_generated.xml"));
            if (xmlIn.Exists) xmlIn.Delete();
            ob.Export(xmlIn, ExportOptions.WithDefaults);
            var doc = new XmlDocument(); doc.Load(xmlIn.FullName);
            XmlNode attrs = null;
            foreach (XmlNode n in doc.GetElementsByTagName("AttributeList")) { attrs = n; break; }
            string before = "";
            foreach (XmlNode c in attrs.ChildNodes) if (c.LocalName == "SecondaryType" || c.LocalName == "Number") before += c.LocalName + "=" + c.InnerText + " ";
            sb.AppendLine("generated OB attributes: " + before);
            Set(doc, attrs, "AutoNumber", "false");
            Set(doc, attrs, "Number", "30");
            Set(doc, attrs, "SecondaryType", "CyclicInterrupt");
            // Program-cycle interface (Initial_Call, Remanence) -> cyclic-interrupt interface
            // (Initial_Call, Event_Count)
            foreach (XmlNode m in doc.GetElementsByTagName("Member")) {
                var e = (XmlElement)m;
                if (e.GetAttribute("Name") == "Remanence") {
                    e.SetAttribute("Name", "Event_Count");
                    e.SetAttribute("Datatype", "Int");
                    foreach (XmlNode t in e.GetElementsByTagName("MultiLanguageText"))
                        t.InnerText = "Number of lost call events";
                }
            }
            // CyclicTime / PhaseShift: SimaticML wants the attribute list in alphabetical
            // order; which ReadOnly flag V18 accepts is tried in turn.
            string[] variants = { "true" };  // V18: "false"/none -> "missing ReadOnly"
            bool ok = false;
            foreach (var ro in variants) {
                var d2 = (XmlDocument)doc.Clone();
                XmlNode a2 = null;
                foreach (XmlNode n in d2.GetElementsByTagName("AttributeList")) { a2 = n; break; }
                foreach (var pair in new[] { new[] { "CyclicTime", "100000" } }) {  // PhaseShift: "not supported" on import, default 0
                    var el = d2.CreateElement(pair[0], a2.NamespaceURI);
                    el.InnerText = pair[1];
                    if (ro != "") el.SetAttribute("ReadOnly", ro);
                    XmlNode after = null;
                    foreach (XmlNode c in a2.ChildNodes)
                        if (c.NodeType == XmlNodeType.Element && string.CompareOrdinal(c.LocalName, pair[0]) < 0) after = c;
                    if (after == null) a2.PrependChild(el); else a2.InsertAfter(el, after);
                }
                var xmlOut = new FileInfo(Path.Combine(workDir, "Cyclic_Pressure_OB30_ro-" + (ro == "" ? "none" : ro) + ".xml"));
                d2.Save(xmlOut.FullName);
                try {
                    plc.BlockGroup.Blocks.Import(xmlOut, ImportOptions.Override);
                    sb.AppendLine("import OK with ReadOnly=" + (ro == "" ? "(none)" : ro) + ": " + xmlOut.Name);
                    ok = true; break;
                } catch (Exception ex) {
                    sb.AppendLine("import failed with ReadOnly=" + (ro == "" ? "(none)" : ro) + ": " + (ex.InnerException ?? ex).Message);
                }
            }
            if (!ok) { project.Close(); return sb.ToString(); }
            var ob30 = plc.BlockGroup.Blocks.Find("Cyclic_Pressure");
            sb.AppendLine("imported " + ob30.Name + " number=" + ob30.Number + " type=" + ob30.GetType().Name
                + " SecondaryType=" + ob30.GetAttribute("SecondaryType") + " CyclicTime=" + ob30.GetAttribute("CyclicTime") + " us");
            try {
                if (Convert.ToInt32(ob30.GetAttribute("CyclicTime")) != 100000) {
                    ob30.SetAttribute("CyclicTime", 100000);
                    sb.AppendLine("CyclicTime set to " + ob30.GetAttribute("CyclicTime") + " us");
                }
            } catch (Exception ex) { sb.AppendLine("CyclicTime read/set: " + ex.Message); }

            var result = plc.GetService<ICompilable>().Compile();
            sb.AppendLine("COMPILE " + result.State + "  errors=" + result.ErrorCount + "  warnings=" + result.WarningCount);
            foreach (var m in result.Messages) Walk(m, sb, 1);
            project.Save();
            sb.AppendLine("project: " + project.Path.FullName);
            project.Close();
        }
        return sb.ToString();
    }
}
"@
Add-Type -TypeDefinition $code -ReferencedAssemblies @($api, "System.Xml")
$work = Join-Path $root "results\tia_xml"
New-Item -ItemType Directory -Force $work | Out-Null
$srcs = (Join-Path $root "plc\tia\FC_LoadParams.scl") + ";" + (Join-Path $root "plc\tia\LineIO_OB_DB.scl")
$text = [XtrOb30]::Run($Project, $srcs, $work)
$text
$text | Out-File -Encoding utf8 (Join-Path $root "results\tia_compile_ob30.txt")
