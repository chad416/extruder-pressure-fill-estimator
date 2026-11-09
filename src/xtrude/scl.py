"""Run the SCL source files of this repo in Python (for tests and CI).

This is a small translator for the subset of Siemens SCL used in plc/*.scl.
It is NOT a general SCL compiler.  It parses each FUNCTION_BLOCK, turns it
into a Python class and runs it with S7 data types:

* Real  -> numpy.float32 (IEEE single, rounded after every operation)
* Int   -> 16-bit, DInt -> 32-bit (wrap-around on assignment)
* Bool, Array[lo..hi] of <elementary>, multi-instances of other FBs

Supported statements: assignment, IF/ELSIF/ELSE, FOR ... TO ... [BY] ... DO,
REGION, RETURN, FB multi-instance calls.  Expressions: arithmetic, comparison,
AND/OR/XOR/NOT, MOD, array index, member access, and the functions LIMIT, MIN,
MAX, ABS, SQRT, EXP, LN, INT_TO_REAL, DINT_TO_REAL, REAL_TO_DINT, INT_TO_DINT.

Operation counting (for the PLC cycle-time budget, dossier section 7):
- every Real +,-,*,/ is counted as "real";
- every Real comparison as "cmp";
- every EXP/LN/SQRT as "math";
- every assignment as "mov";
- every integer +,-,* and every loop pass as "int".
`OPS` holds the counts; reset it with `reset_ops()`.

Origin: copied from the author's tcu-zone-pid-tuning project (same author, MIT), then
extended with EXP/LN and the counters.

Anything else raises SclError, so an unsupported construct can never be
silently ignored.  The authority for the real PLC is the TIA Portal compiler;
this file only lets CI execute the same source text that goes into TIA.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np


class SclError(Exception):
    pass


# --------------------------------------------------------------------------
# lexer
# --------------------------------------------------------------------------

_TOKEN_RE = re.compile(r"""
    (?P<ws>\s+)
  | (?P<comment>//[^\n]*|\(\*.*?\*\)|/\*.*?\*/)
  | (?P<attr>\{[^}]*\})
  | (?P<real>\d+\.\d+(?:[eE][+-]?\d+)?|\d+[eE][+-]?\d+)
  | (?P<int>\d+)
  | (?P<local>\#[A-Za-z_][A-Za-z0-9_]*)
  | (?P<qname>"[^"]+")
  | (?P<ident>[A-Za-z_][A-Za-z0-9_]*)
  | (?P<op>:=|=>|<>|<=|>=|\.\.|[-+*/=<>();:,\[\].])
""", re.VERBOSE | re.DOTALL)

_HEADER_RE = re.compile(r"^\s*(TITLE\s*=|AUTHOR\s*:|VERSION\s*:|FAMILY\s*:)[^\n]*$",
                        re.IGNORECASE | re.MULTILINE)


@dataclass
class Tok:
    kind: str
    text: str
    line: int

    @property
    def up(self) -> str:
        return self.text.upper()


def tokenize(src: str) -> list[Tok]:
    src = _HEADER_RE.sub("", src)
    toks: list[Tok] = []
    pos, line = 0, 1
    while pos < len(src):
        m = _TOKEN_RE.match(src, pos)
        if not m:
            raise SclError(f"line {line}: cannot read {src[pos:pos + 20]!r}")
        kind = m.lastgroup
        text = m.group()
        if kind not in ("ws", "comment", "attr"):
            if kind == "local":
                text = text[1:]
            elif kind == "qname":
                text = text[1:-1]
            toks.append(Tok(kind, text, line))
        line += text.count("\n") if kind in ("ws", "comment", "attr") else 0
        pos = m.end()
    toks.append(Tok("eof", "", line))
    return toks


# --------------------------------------------------------------------------
# parser -> Python source
# --------------------------------------------------------------------------

ELEMENTARY = {"REAL", "INT", "DINT", "BOOL"}

#: Names TIA Portal rejects as variable names (data types and keywords).  Found the
#: hard way: an output called "Time" passed this translator but not the TIA compiler.
RESERVED = {
    "TIME", "LTIME", "S5TIME", "DATE", "TOD", "LTOD", "TIME_OF_DAY", "DT", "LDT", "DTL",
    "DATE_AND_TIME", "STRING", "WSTRING", "CHAR", "WCHAR", "BYTE", "WORD", "DWORD", "LWORD",
    "BOOL", "SINT", "INT", "DINT", "LINT", "USINT", "UINT", "UDINT", "ULINT", "REAL", "LREAL",
    "ARRAY", "STRUCT", "VOID", "ANY", "POINTER", "VARIANT", "IF", "THEN", "ELSIF", "ELSE",
    "END_IF", "CASE", "OF", "END_CASE", "FOR", "TO", "BY", "DO", "END_FOR", "WHILE",
    "END_WHILE", "REPEAT", "UNTIL", "END_REPEAT", "EXIT", "CONTINUE", "RETURN", "GOTO",
    "AND", "OR", "XOR", "NOT", "MOD", "TRUE", "FALSE", "VAR", "BEGIN", "REGION", "END_REGION",
}


@dataclass
class VarDecl:
    name: str
    section: str  # VAR_INPUT / VAR_OUTPUT / VAR / VAR_TEMP / VAR_IN_OUT
    type: str  # REAL / INT / DINT / BOOL / ARRAY / FB:<name>
    elem: str = ""
    lo: int = 0
    hi: int = 0
    init: object = None


class Parser:
    def __init__(self, toks: list[Tok], fb_types: dict[str, dict[str, VarDecl]]):
        self.t = toks
        self.i = 0
        self.fb_types = fb_types  # name -> its variable table (for member types)
        self.vars: dict[str, VarDecl] = {}
        self.lines: list[str] = []
        self.loop_id = 0

    # -- token helpers ------------------------------------------------------
    def peek(self, k: int = 0) -> Tok:
        return self.t[self.i + k]

    def next(self) -> Tok:
        tok = self.t[self.i]
        self.i += 1
        return tok

    def accept(self, text: str) -> bool:
        if self.peek().up == text.upper() and self.peek().kind in ("ident", "op"):
            self.i += 1
            return True
        return False

    def expect(self, text: str) -> Tok:
        tok = self.next()
        if tok.up != text.upper():
            raise SclError(f"line {tok.line}: expected {text!r}, got {tok.text!r}")
        return tok

    # -- declarations -------------------------------------------------------
    def parse_type(self) -> tuple[str, str, int, int]:
        tok = self.next()
        if tok.kind == "qname":
            if tok.text not in self.fb_types:
                raise SclError(f"line {tok.line}: unknown FB type {tok.text!r}")
            return "FB:" + tok.text, "", 0, 0
        if tok.up == "ARRAY":
            self.expect("[")
            lo = self.parse_int_const()
            self.expect("..")
            hi = self.parse_int_const()
            self.expect("]")
            self.expect("OF")
            elem = self.next().up
            if elem not in ELEMENTARY:
                raise SclError(f"line {tok.line}: unsupported array element {elem}")
            return "ARRAY", elem, lo, hi
        if tok.up not in ELEMENTARY:
            raise SclError(f"line {tok.line}: unsupported type {tok.text!r}")
        return tok.up, "", 0, 0

    def parse_int_const(self) -> int:
        sign = -1 if self.accept("-") else 1
        tok = self.next()
        if tok.kind != "int":
            raise SclError(f"line {tok.line}: expected integer, got {tok.text!r}")
        return sign * int(tok.text)

    def parse_literal(self) -> object:
        sign = -1 if self.accept("-") else 1
        tok = self.next()
        if tok.kind == "real":
            return sign * float(tok.text)
        if tok.kind == "int":
            return sign * int(tok.text)
        if tok.up in ("TRUE", "FALSE"):
            return tok.up == "TRUE"
        raise SclError(f"line {tok.line}: unsupported initial value {tok.text!r}")

    def parse_var_sections(self) -> None:
        while self.peek().up.startswith("VAR"):
            section = self.next().up
            if self.accept("CONSTANT"):
                raise SclError("VAR CONSTANT is not supported")
            while not self.accept("END_VAR"):
                name_tok = self.next()
                if name_tok.kind != "ident":
                    raise SclError(f"line {name_tok.line}: bad variable name {name_tok.text!r}")
                if name_tok.up in RESERVED:
                    raise SclError(f"line {name_tok.line}: {name_tok.text!r} is a reserved word in SCL")
                self.expect(":")
                typ, elem, lo, hi = self.parse_type()
                init = None
                if self.accept(":="):
                    init = self.parse_literal()
                self.expect(";")
                self.vars[name_tok.text] = VarDecl(name_tok.text, section, typ, elem, lo, hi, init)

    # -- types of expressions -------------------------------------------------
    @staticmethod
    def num_type(*types: str) -> str:
        if "REAL" in types:
            return "REAL"
        if "DINT" in types:
            return "DINT"
        return "INT"

    # -- expressions: return (python_source, scl_type) -----------------------
    def expr(self) -> tuple[str, str]:
        return self.or_expr()

    def _binary(self, sub, ops: dict[str, str]) -> tuple[str, str]:
        left, lt = sub()
        while self.peek().up in ops and self.peek().kind in ("op", "ident"):
            op = self.next().up
            right, rt = sub()
            left, lt = self.combine(op, ops[op], left, lt, right, rt)
        return left, lt

    def combine(self, op: str, pyop: str, a: str, at: str, b: str, bt: str) -> tuple[str, str]:
        if op in ("AND", "OR", "XOR", "&"):
            if at != "BOOL" or bt != "BOOL":
                raise SclError(f"{op} on non-Bool operands is not supported")
            return f"({a} {pyop} {b})", "BOOL"
        if op in ("=", "<>", "<", ">", "<=", ">="):
            if "REAL" in (at, bt):
                return f"_cmp({a} {pyop} {b})", "BOOL"
            return f"({a} {pyop} {b})", "BOOL"
        t = self.num_type(at, bt)
        if op == "/" and t != "REAL":
            return f"_idiv({a}, {b})", t
        if op == "MOD":
            if t == "REAL":
                raise SclError("MOD on Real is not supported")
            return f"_imod({a}, {b})", t
        if t == "REAL":
            return f"_fop(_f32({a}) {pyop} _f32({b}))", "REAL"
        return f"_iop({a} {pyop} {b})", t

    def or_expr(self):
        return self._binary(self.xor_expr, {"OR": "or"})

    def xor_expr(self):
        return self._binary(self.and_expr, {"XOR": "!="})

    def and_expr(self):
        return self._binary(self.eq_expr, {"AND": "and", "&": "and"})

    def eq_expr(self):
        return self._binary(self.rel_expr, {"=": "==", "<>": "!="})

    def rel_expr(self):
        return self._binary(self.add_expr, {"<": "<", ">": ">", "<=": "<=", ">=": ">="})

    def add_expr(self):
        return self._binary(self.mul_expr, {"+": "+", "-": "-"})

    def mul_expr(self):
        return self._binary(self.unary, {"*": "*", "/": "/", "MOD": "%"})

    def unary(self) -> tuple[str, str]:
        if self.accept("-"):
            s, t = self.unary()
            return f"(-{s})", t
        if self.accept("+"):
            return self.unary()
        if self.accept("NOT"):
            s, t = self.unary()
            if t != "BOOL":
                raise SclError("NOT on non-Bool is not supported")
            return f"(not {s})", "BOOL"
        return self.primary()

    def primary(self) -> tuple[str, str]:
        tok = self.next()
        if tok.kind == "real":
            return f"_f32({float(tok.text)!r})", "REAL"
        if tok.kind == "int":
            return tok.text, "INT"
        if tok.up in ("TRUE", "FALSE"):
            return ("True" if tok.up == "TRUE" else "False"), "BOOL"
        if tok.text == "(":
            s, t = self.expr()
            self.expect(")")
            return f"({s})", t
        if tok.kind == "local":
            return self.access(tok)
        if tok.kind == "ident" and self.peek().text == "(":
            return self.function_call(tok)
        raise SclError(f"line {tok.line}: unexpected {tok.text!r} in expression")

    def access(self, tok: Tok) -> tuple[str, str]:
        if tok.text not in self.vars:
            raise SclError(f"line {tok.line}: undeclared #{tok.text}")
        d = self.vars[tok.text]
        src, typ = f"self.{d.name}", d.type
        if self.peek().text == "[":
            if d.type != "ARRAY":
                raise SclError(f"line {tok.line}: #{d.name} is not an array")
            self.next()
            idx, it = self.expr()
            self.expect("]")
            if it not in ("INT", "DINT"):
                raise SclError(f"line {tok.line}: array index must be integer")
            src, typ = f"self.{d.name}[_ix({idx}, {d.lo}, {d.hi})]", d.elem
        elif self.peek().text == ".":
            if not d.type.startswith("FB:"):
                raise SclError(f"line {tok.line}: #{d.name} has no members")
            self.next()
            member = self.next().text
            fb_vars = self.fb_types[d.type[3:]]
            if member not in fb_vars or fb_vars[member].section not in ("VAR_OUTPUT", "VAR_INPUT"):
                raise SclError(f"line {tok.line}: {d.name}.{member} is not an input/output")
            src, typ = f"self.{d.name}.{member}", fb_vars[member].type
        return src, typ

    def function_call(self, tok: Tok) -> tuple[str, str]:
        name = tok.up
        self.expect("(")
        args: dict[str, tuple[str, str]] = {}
        pos: list[tuple[str, str]] = []
        while not self.accept(")"):
            if self.peek().kind == "ident" and self.peek(1).text == ":=":
                key = self.next().up
                self.next()
                args[key] = self.expr()
            else:
                pos.append(self.expr())
            self.accept(",")
        if name == "LIMIT":
            mn, inp, mx = (args["MN"], args["IN"], args["MX"]) if args else pos
            t = self.num_type(mn[1], inp[1], mx[1])
            cast = "_f32" if t == "REAL" else ""
            return f"_limit({cast}({mn[0]}), {cast}({inp[0]}), {cast}({mx[0]}))", t
        if name in ("MIN", "MAX"):
            vals = [args[k] for k in sorted(args)] if args else pos
            t = self.num_type(*(v[1] for v in vals))
            cast = "_f32" if t == "REAL" else ""
            inner = ", ".join(f"{cast}({v[0]})" for v in vals)
            return f"{name.lower()}({inner})", t
        one = (args.get("IN") or pos[0]) if (args or pos) else None
        if one is None:
            raise SclError(f"line {tok.line}: {name} needs an argument")
        if name == "ABS":
            return f"abs({one[0]})", one[1]
        if name == "SQRT":
            return f"_fn(np.sqrt(_f32({one[0]})))", "REAL"
        if name == "EXP":
            return f"_fn(np.exp(_f32({one[0]})))", "REAL"
        if name == "LN":
            return f"_fn(np.log(_f32({one[0]})))", "REAL"
        if name in ("INT_TO_REAL", "DINT_TO_REAL"):
            return f"_f32({one[0]})", "REAL"
        if name == "INT_TO_DINT":
            return f"int({one[0]})", "DINT"
        if name == "REAL_TO_DINT":
            return f"_round_even({one[0]})", "DINT"
        raise SclError(f"line {tok.line}: function {name} is not supported")

    # -- statements -----------------------------------------------------------
    def emit(self, depth: int, text: str) -> None:
        self.lines.append("    " * depth + text)

    def statements(self, depth: int, stop: tuple[str, ...]) -> None:
        start = len(self.lines)
        while self.peek().up not in stop:
            self.statement(depth)
        if len(self.lines) == start:
            self.emit(depth, "pass")

    def cast_for(self, typ: str, src: str) -> str:
        if typ == "REAL":
            return f"_f32({src})"
        if typ == "INT":
            return f"_wrap({src}, 16)"
        if typ == "DINT":
            return f"_wrap({src}, 32)"
        if typ == "BOOL":
            return f"bool({src})"
        raise SclError(f"cannot assign to type {typ}")

    def statement(self, depth: int) -> None:
        tok = self.peek()
        up = tok.up
        if tok.text == ";":
            self.next()
            return
        if up == "REGION":
            self.next()
            self.next()  # region name (single identifier)
            self.statements(depth, ("END_REGION",))
            self.expect("END_REGION")
            return
        if up == "IF":
            self.next()
            cond, ct = self.expr()
            if ct != "BOOL":
                raise SclError(f"line {tok.line}: IF condition is not Bool")
            self.expect("THEN")
            self.emit(depth, f"if {cond}:")
            self.statements(depth + 1, ("ELSIF", "ELSE", "END_IF"))
            while self.accept("ELSIF"):
                cond, _ = self.expr()
                self.expect("THEN")
                self.emit(depth, f"elif {cond}:")
                self.statements(depth + 1, ("ELSIF", "ELSE", "END_IF"))
            if self.accept("ELSE"):
                self.emit(depth, "else:")
                self.statements(depth + 1, ("END_IF",))
            self.expect("END_IF")
            self.expect(";")
            return
        if up == "FOR":
            self.next()
            var = self.next()
            if var.kind != "local" or self.vars.get(var.text, VarDecl("", "", "")).type not in ("INT", "DINT"):
                raise SclError(f"line {var.line}: FOR needs an Int/DInt #variable")
            self.expect(":=")
            a, _ = self.expr()
            self.expect("TO")
            b, _ = self.expr()
            step = "1"
            if self.accept("BY"):
                step, _ = self.expr()
            self.expect("DO")
            self.loop_id += 1
            end, stp = f"_end{self.loop_id}", f"_stp{self.loop_id}"
            v = f"self.{var.text}"
            self.emit(depth, f"{v} = {a}; {end} = {b}; {stp} = {step}")
            self.emit(depth, f"while ({v} <= {end}) if {stp} > 0 else ({v} >= {end}):")
            self.statements(depth + 1, ("END_FOR",))
            self.emit(depth + 1, f"{v} = _iop({v} + {stp})")
            self.expect("END_FOR")
            self.expect(";")
            return
        if up == "RETURN":
            self.next()
            self.expect(";")
            self.emit(depth, "return")
            return
        if tok.kind == "local":
            self.next()
            d = self.vars.get(tok.text)
            if d is None:
                raise SclError(f"line {tok.line}: undeclared #{tok.text}")
            if self.peek().text == "(":
                self.fb_call(d, depth)
                return
            if self.peek().text == "[":
                self.next()
                idx, _ = self.expr()
                self.expect("]")
                target, typ = f"self.{d.name}[_ix({idx}, {d.lo}, {d.hi})]", d.elem
            else:
                if d.type == "ARRAY" or d.type.startswith("FB:"):
                    raise SclError(f"line {tok.line}: whole-array/instance assignment not supported")
                if d.section == "VAR_INPUT":
                    raise SclError(f"line {tok.line}: writing input #{d.name} is not allowed here")
                target, typ = f"self.{d.name}", d.type
            self.expect(":=")
            val, _ = self.expr()
            self.expect(";")
            self.emit(depth, f"{target} = _mov({self.cast_for(typ, val)})")
            return
        raise SclError(f"line {tok.line}: unsupported statement starting with {tok.text!r}")

    def fb_call(self, d: VarDecl, depth: int) -> None:
        if not d.type.startswith("FB:"):
            raise SclError(f"#{d.name} is not an FB instance")
        fb_vars = self.fb_types[d.type[3:]]
        self.expect("(")
        while not self.accept(")"):
            pname = self.next().text
            if pname not in fb_vars or fb_vars[pname].section != "VAR_INPUT":
                raise SclError(f"{d.name}: {pname} is not an input of {d.type[3:]}")
            self.expect(":=")
            val, _ = self.expr()
            self.emit(depth, f"self.{d.name}.{pname} = {self.cast_for(fb_vars[pname].type, val)}")
            self.accept(",")
        self.expect(";")
        self.emit(depth, f"self.{d.name}()")


# --------------------------------------------------------------------------
# runtime helpers used by the generated code
# --------------------------------------------------------------------------

def _f32(x) -> np.float32:
    return np.float32(x)


OPS = {"real": 0, "cmp": 0, "math": 0, "mov": 0, "int": 0}


def reset_ops() -> None:
    for k in OPS:
        OPS[k] = 0


def _fop(x):
    OPS["real"] += 1
    return np.float32(x)


def _fn(x):
    OPS["math"] += 1
    return np.float32(x)


def _cmp(x):
    OPS["cmp"] += 1
    return x


def _mov(x):
    OPS["mov"] += 1
    return x


def _iop(x):
    OPS["int"] += 1
    return x


def _wrap(x, bits: int) -> int:
    x = int(x)
    m = 1 << bits
    x &= m - 1
    return x - m if x >= m >> 1 else x


def _idiv(a: int, b: int) -> int:
    if b == 0:
        raise ZeroDivisionError("integer division by zero")
    q = abs(int(a)) // abs(int(b))
    return q if (a >= 0) == (b >= 0) else -q


def _imod(a: int, b: int) -> int:
    return int(a) - _idiv(a, b) * int(b)


def _limit(mn, x, mx):
    return mn if x < mn else (mx if x > mx else x)


def _ix(i: int, lo: int, hi: int) -> int:
    if not lo <= i <= hi:
        raise IndexError(f"array index {i} outside [{lo}..{hi}]")
    return i - lo


def _round_even(x) -> int:
    return int(np.rint(np.float32(x)))


_DEFAULT = {"REAL": "np.float32(0.0)", "INT": "0", "DINT": "0", "BOOL": "False"}


# --------------------------------------------------------------------------
# public API
# --------------------------------------------------------------------------

def _block_spans(toks: list[Tok]) -> list[tuple[str, int, int]]:
    spans, i = [], 0
    while toks[i].kind != "eof":
        if toks[i].up == "FUNCTION_BLOCK":
            name = toks[i + 1].text
            j = i
            while toks[j].up != "END_FUNCTION_BLOCK":
                if toks[j].kind == "eof":
                    raise SclError(f"{name}: missing END_FUNCTION_BLOCK")
                j += 1
            spans.append((name, i, j))
            i = j + 1
        elif toks[i].up in ("ORGANIZATION_BLOCK", "FUNCTION", "DATA_BLOCK", "TYPE"):
            raise SclError(f"line {toks[i].line}: only FUNCTION_BLOCKs are translated")
        else:
            raise SclError(f"line {toks[i].line}: unexpected {toks[i].text!r}")
    return spans


def translate(sources: list[str]) -> tuple[str, dict[str, dict[str, VarDecl]]]:
    """Translate SCL sources (any order) into one Python module source."""
    blocks = []
    for src in sources:
        toks = tokenize(src)
        for name, a, b in _block_spans(toks):
            blocks.append((name, toks[a:b + 1] + [Tok("eof", "", toks[b].line)]))
    fb_types: dict[str, dict[str, VarDecl]] = {}
    code = ["import numpy as np",
            "from xtrude.scl import _f32, _wrap, _idiv, _imod, _limit, _ix, _round_even, _fop, _fn, _cmp, _mov, _iop",
            ""]
    pending = dict(blocks)
    while pending:  # a block can only be translated after the FBs it instantiates
        progressed = False
        for name, toks in list(pending.items()):
            p = Parser(toks, fb_types)
            p.expect("FUNCTION_BLOCK")
            p.next()
            try:
                p.parse_var_sections()
            except SclError as e:
                if "unknown FB type" in str(e):
                    continue
                raise
            p.expect("BEGIN")
            p.statements(2, ("END_FUNCTION_BLOCK",))
            p.expect("END_FUNCTION_BLOCK")
            fb_types[name] = p.vars
            cls = _class_source(name, p)
            code.extend(cls)
            del pending[name]
            progressed = True
        if not progressed:
            raise SclError(f"cannot resolve FB types for: {', '.join(pending)}")
    return "\n".join(code) + "\n", fb_types


def _class_source(name: str, p: Parser) -> list[str]:
    cname = re.sub(r"\W", "_", name)
    out = [f"class {cname}:", f"    SCL_NAME = {name!r}", "    def __init__(self):"]
    temps = []
    for d in p.vars.values():
        if d.type.startswith("FB:"):
            init = f"_FB[{d.type[3:]!r}]()"
        elif d.type == "ARRAY":
            init = f"[{_DEFAULT[d.elem]} for _ in range({d.hi - d.lo + 1})]"
        elif d.init is not None:
            init = {"REAL": f"np.float32({float(d.init)!r})", "BOOL": repr(bool(d.init))}.get(d.type, repr(int(d.init)))
        else:
            init = _DEFAULT[d.type]
        out.append(f"        self.{d.name} = {init}")
        if d.section == "VAR_TEMP":
            temps.append((d, init))
    out.append("    def __call__(self):")
    for d, init in temps:  # temp data is re-initialised on every call
        out.append(f"        self.{d.name} = {init}")
    out.extend(p.lines)
    out.append(f"_FB[{name!r}] = {cname}")
    out.append("")
    return out


def load(paths: list[str | Path]) -> dict[str, type]:
    """Translate and load SCL files; returns {FB name: Python class}."""
    sources = [Path(x).read_text(encoding="utf-8") for x in paths]
    code, _ = translate(sources)
    namespace: dict[str, object] = {"_FB": {}}
    exec(compile(code, "<scl>", "exec"), namespace)
    return dict(namespace["_FB"])  # type: ignore[arg-type]


def generated_source(paths: list[str | Path]) -> str:
    return translate([Path(x).read_text(encoding="utf-8") for x in paths])[0]


__all__ = ["SclError", "load", "translate", "generated_source", "math", "OPS", "reset_ops"]
