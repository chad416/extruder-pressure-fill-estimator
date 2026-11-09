<#
  Evidence for SPEC V02: open the built project in TIA Portal WITH its user interface,
  show a block in the editor, compile, and save a PNG of the TIA window.

  Usage: powershell -ExecutionPolicy Bypass -File scripts\tia_screenshot.ps1 -Project <path.ap18> -Block FB_PressureRST -Out docs\img\tia_FB_PressureRST.png
#>
param(
    [Parameter(Mandatory = $true)][string]$Project,
    [string]$Block = "FB_PressureRST",
    [Parameter(Mandatory = $true)][string]$Out,
    [string]$Version = "V18",
    [int]$SettleSeconds = 25
)
$ErrorActionPreference = "Stop"
$api = "C:\Program Files\Siemens\Automation\Portal $Version\PublicAPI\$Version\Siemens.Engineering.dll"
$portal = "C:\Program Files\Siemens\Automation\Portal $Version"

$resolver = @"
using System; using System.IO; using System.Reflection;
public static class OpnResolve3 {
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
[OpnResolve3]::Init(@((Split-Path -Parent $api), "$portal\Bin\PublicAPI", "$portal\Bin"))
[void][System.Reflection.Assembly]::LoadFrom($api)

$code = @"
using System;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;
using Siemens.Engineering;
using Siemens.Engineering.HW;
using Siemens.Engineering.HW.Features;
using Siemens.Engineering.SW;
using Siemens.Engineering.Compiler;

public static class XtrShot {
    [DllImport("user32.dll")] static extern bool ShowWindow(IntPtr h, int n);
    [DllImport("user32.dll")] static extern bool GetWindowRect(IntPtr h, out RECT r);
    [DllImport("user32.dll")] static extern bool PrintWindow(IntPtr h, IntPtr hdc, uint flags);
    [StructLayout(LayoutKind.Sequential)] public struct RECT { public int L, T, R, B; }

    public static string Run(string projectPath, string block, string outPng, int settle) {
        using (var tia = new TiaPortal(TiaPortalMode.WithUserInterface)) {
            var project = tia.Projects.Open(new FileInfo(projectPath));
            PlcSoftware plc = null;
            foreach (var dev in project.Devices)
                foreach (var item in dev.DeviceItems) {
                    var sc = item.GetService<SoftwareContainer>();
                    if (sc != null && sc.Software is PlcSoftware) plc = (PlcSoftware)sc.Software;
                }
            var b = plc.BlockGroup.Blocks.Find(block);
            b.ShowInEditor();
            var res = plc.GetService<ICompilable>().Compile();
            Thread.Sleep(settle * 1000);
            var pid = tia.GetCurrentProcess().Id;
            var h = Process.GetProcessById(pid).MainWindowHandle;
            ShowWindow(h, 3);  // maximise
            Thread.Sleep(4000);
            RECT r; GetWindowRect(h, out r);
            using (var bmp = new Bitmap(r.R - r.L, r.B - r.T)) {
                using (var g = Graphics.FromImage(bmp)) {
                    var hdc = g.GetHdc();
                    PrintWindow(h, hdc, 2);  // PW_RENDERFULLCONTENT
                    g.ReleaseHdc(hdc);
                }
                bmp.Save(outPng, ImageFormat.Png);
            }
            var msg = "compile " + res.State + " errors=" + res.ErrorCount + " warnings=" + res.WarningCount + "; screenshot " + outPng;
            project.Close();
            return msg;
        }
    }
}
"@
Add-Type -TypeDefinition $code -ReferencedAssemblies @($api, "System.Drawing")
[XtrShot]::Run($Project, $Block, [System.IO.Path]::GetFullPath($Out), $SettleSeconds)
