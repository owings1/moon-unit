from __future__ import annotations

import array
import struct
from collections import namedtuple

try:
  from typing import Any, Callable, TypeVar
  _T = TypeVar('_T')
except ImportError:
  pass
try:
  from micropython import const
except ImportError:
  # Support generic environment
  def const(x: _T): return x

MAX_MOTORS = const(0x02)
MOTOR_BLOCK_SIZE = const(0x40)
SCRIPT_PAGE_SIZE = const(0xF8)
NUM_SCRIPT_PAGES = const(0x08)
NUM_SCRIPT_GLOBAL_VARS = const(0x04)
NUM_SCRIPT_LOCAL_VARS = const(0x04)
SCRIPT_LOCAL_VAR_OFFSET = const(0x10)
SCRIPT_STACK_SIZE = const(0x08)
BUSY_EXEMPT_MASK = const(0x80)

INDIRECT_OPCODE_FLAG = const(0x40)
FARPTR_OPCODE_FLAG = const(0x80)
CONTROL_EXCODE = const(0x40|0x80)
FUNC_NEGATED_FLAG = const(0x80)
FUNC_NEGATED_BIT = const(7)
INDPTR_END = const(0x1C)
GLOBAL_VARPTR_START = const(0x60)
LOCAL_VARPTR_START = const(0x70)
PAGE_REGISTER = const(0x04)
SCRIPT_PAGE_START = const(0x10)
MOTOR_BASE_ADDR = const(0x08)

GlobalVar = range(NUM_SCRIPT_GLOBAL_VARS)
GlobalVarPtr = range(GLOBAL_VARPTR_START, GLOBAL_VARPTR_START + NUM_SCRIPT_GLOBAL_VARS)
LocalVar = range(SCRIPT_LOCAL_VAR_OFFSET, SCRIPT_LOCAL_VAR_OFFSET + NUM_SCRIPT_LOCAL_VARS)
LocalVarPtr = range(LOCAL_VARPTR_START, LOCAL_VARPTR_START + NUM_SCRIPT_LOCAL_VARS)

class WriteSource:
  VMEXC = 0x00
  BUSIO = 0x01

  @classmethod
  def attrs(cls, source: int) -> list[Attribute]:
    return sorted((
      attr for attr in attrsmap.values()
      if attr.writewhen & (1 << source)),
      key=lambda x: x.offset)

  @classmethod
  def mask(cls, source: int) -> int:
    mask = 0
    for attr in cls.attrs(source):
      mask |= attr.offsets_mask()
    return mask

  @classmethod
  def maskinfos(cls) -> list[str]:
    names = tuple(x for x in dir(cls) if x.upper() == x)
    values = list(getattr(cls, x) for x in names)
    values.append(7) # Bit 7 is busy exempt
    masks = list(map(cls.mask, values))
    varnames = list(f'{name}_write_mask'.upper() for name in names)
    varnames.append('busy_write_mask'.upper())
    hexstrs = list(f'0x{x:016X}' for x in masks)
    cpplines = list(
      f'const uint64_t {varname} = {hexstr};'
      for (varname, hexstr) in zip(varnames, hexstrs))
    def attrlines(source):
      for attr in cls.attrs(source):
        yield f'// {attr.textrow()}'
    attrsblocks = ('\n'.join(attrlines(source)) for source in values)
    entries = list(zip(values, attrsblocks, cpplines))
    entries.sort(key=lambda x: x[0])
    return ['\n'.join(x[1:]) for x in entries]

class WriteWhen:
  NEVER = 0x00
  SRC_VMEXC = 1 << WriteSource.VMEXC
  SRC_BUSIO = 1 << WriteSource.BUSIO
  SRC_ANY = SRC_VMEXC|SRC_BUSIO
  BUSY = 0x80
  ALWAYS = SRC_ANY|BUSY

class Attribute(namedtuple('Attribute', ('offset', 'fmt', 'size', 'writewhen'))):
  offset: int
  fmt: bytes
  size: int
  writewhen: int

  @property
  def name(self) -> str:
    return revoffs[self.offset]

  def offsets(self) -> range:
    return range(self.offset, self.offset + self.size)

  def offsets_mask(self) -> int:
    mask = 0
    for offset in self.offsets():
      mask |= 1 << offset
    return mask

  def textrow(self) -> str:
    return ' | '.join((
      f'{self.name:18}',
      f'0x{self.offset:02X}',
      f'{self.size} bytes'))

class Flag(namedtuple('Flag', ('bit', 'attr'))):
  bit: int
  attr: Attribute

class Attributes:
  state_flags = Attribute(0x00, b'B', 1, WriteWhen.NEVER)
  script_page = Attribute(0x01, b'B', 1, WriteWhen.NEVER)
  script_index = Attribute(0x02, b'B', 1, WriteWhen.NEVER)
  script_repcode = Attribute(0x03, b'B', 1, WriteWhen.NEVER)
  current_position = Attribute(0x04, b'l', 4, WriteWhen.SRC_ANY)
  target_position = Attribute(0x08, b'l', 4, WriteWhen.NEVER)
  speed = Attribute(0x0C, b'f', 4, WriteWhen.NEVER)
  settings_flags = Attribute(0x10, b'B', 1, WriteWhen.SRC_ANY)
  enable_delay_ms = Attribute(0x11, b'B', 1, WriteWhen.SRC_ANY)
  sleep_timeout_ms = Attribute(0x12, b'H', 2, WriteWhen.SRC_ANY)
  max_speed = Attribute(0x14, b'f', 4, WriteWhen.SRC_ANY)
  acceleration = Attribute(0x18, b'f', 4, WriteWhen.SRC_ANY)
  move = Attribute(0x1C, b'l', 4, WriteWhen.SRC_ANY)
  move_to = Attribute(0x20, b'l', 4, WriteWhen.SRC_ANY)
  delay = Attribute(0x24, b'L', 4, WriteWhen.SRC_ANY)
  stop = Attribute(0x28, b'x', 1, WriteWhen.ALWAYS)
  script_clear = Attribute(0x29, b'B', 1, WriteWhen.SRC_BUSIO)
  script_exec = Attribute(0x2A, b'2B', 2, WriteWhen.SRC_BUSIO)
  wait_end_time = Attribute(0x2C, b'L', 4, WriteWhen.NEVER)
  move_rev = Attribute(0x30, b'l', 4, WriteWhen.SRC_ANY)

  @classmethod
  def offsets_writemasks(cls) -> str:
    masks = array.array('B', (0 for _ in range(0x40)))
    for attr in attrsmap.values():
      for offset in attr.offsets():
        masks[offset] = attr.writewhen
    return _cpp_x40map('OFFSET_WRITEMASKS', masks)

  @classmethod
  def datalens(cls) -> str:
    arr = array.array('B', (0 for _ in range(0x40)))
    for attr in attrsmap.values():
      if attr.writewhen & WriteWhen.SRC_VMEXC:
        arr[attr.offset] = attr.size
    return _cpp_x40map('ATTR_DATALENGTHS', arr)

attrsmap: dict[str, Attribute] = {
  name: attr for (name, attr) in
  ((name, getattr(Attributes, name)) for name in dir(Attributes))
  if isinstance(attr, Attribute)}

revoffs: dict[int, str] = {attr.offset: name for name, attr in attrsmap.items()}

revops: dict[int, Attribute] = {
  attr.offset: attr for attr in attrsmap.values()
  if attr.writewhen & WriteWhen.SRC_VMEXC}

opsmap: dict[str, Attribute] = {attr.name: attr for attr in revops.values()}

class CtlOp(namedtuple('CtlOp', ('esc', 'code', 'fmt', 'size', 'prefix_size'))):
  esc: int
  code: int
  fmt: bytes
  size: int
  prefix_size: int|None
  @property
  def name(self) -> str:
    return revctls[self.code]
  
class CtlOps:
  set_var = CtlOp(CONTROL_EXCODE, 0x01, b'Bl', 5, 1)
  var_math1 = CtlOp(CONTROL_EXCODE, 0x02, b'2B', 2, None)
  var_math2 = CtlOp(CONTROL_EXCODE, 0x03, b'2Bl', 6, 2)
  call = CtlOp(CONTROL_EXCODE, 0x08, b'2B', 2, None)
  jump = CtlOp(CONTROL_EXCODE, 0x09, b'2B', 2, None)
  cond_call = CtlOp(CONTROL_EXCODE, 0x0A, b'4B', 4, None)
  cond_jump = CtlOp(CONTROL_EXCODE, 0x0B, b'4B', 4, None)

  @classmethod
  def datalens(cls):
    arr = array.array('B', (0 for _ in range(0x40)))
    for ctlop in ctlopsmap.values():
      arr[ctlop.code] = ctlop.size
    return _cpp_x40map('CTLOP_DATALENGTHS', arr)
    
ctlopsmap: dict[str, CtlOp] = {
  f':{name}': ctlop for (name, ctlop) in
  ((name, getattr(CtlOps, name)) for name in dir(CtlOps))
  if isinstance(ctlop, CtlOp)}

revctls: dict[int, str] = {ctlop.code: name for name, ctlop in ctlopsmap.items()}

revops[CONTROL_EXCODE] = {
  ctlop.code: ctlopsmap[revctls[ctlop.code]] for ctlop in ctlopsmap.values()}
opsmap.update((ctlop.name, ctlop) for ctlop in ctlopsmap.values())

class Flags:
  is_limit_cw = Flag(0, Attributes.state_flags)
  is_limit_acw = Flag(1, Attributes.state_flags)
  is_active = Flag(2, Attributes.state_flags)
  is_moving = Flag(3, Attributes.state_flags)
  is_stopping = Flag(4, Attributes.state_flags)
  is_manual_position = Flag(5, Attributes.state_flags)
  is_script_active = Flag(6, Attributes.state_flags)
  is_delay_active = Flag(7, Attributes.state_flags)
  limits_enabled = Flag(0, Attributes.settings_flags)
  sleep_enabled = Flag(1, Attributes.settings_flags)

P_EQL = const(0b00)
P_LT = const(0b01)
P_AND = const(0b10)
P_LTE = const(0b11)

class FunId:
  AND_STATEFLAGS_RHS = 0x00
  EQL_RETURNCODE_RHS = 0x03
  AND_SETTINGSFLAGS_RHS = 0x10
  P_CALLARG = 0x20
  P_CALLARG_EQL = P_CALLARG|P_EQL
  P_CALLARG_AND = P_CALLARG|P_AND
  P_LASTCONDARG = 0x24
  P_LASTCONDARG_EQL = P_LASTCONDARG|P_EQL
  P_LASTCONDARG_AND = P_LASTCONDARG|P_AND
  P_LASTCOMPARG = 0x28
  P_LASTCOMPARG_EQL = P_LASTCOMPARG|P_EQL
  P_LASTCOMPARG_LT = P_LASTCOMPARG|P_LT
  P_LASTCOMPARG_LTE = P_LASTCOMPARG|P_LTE
  ALWAYS_TRUE = 0xFF >> 1

class Math1Oper:
  MATH1_INC = 0x01
  MATH1_DEC = 0x02
  MATH1_NEG = 0x03
  MATH1_HLF = 0x04
  MATH1_DBL = 0x05

class Math2Oper:
  MATH2_ADD = 0x01
  MATH2_SUB = 0x02
  MATH2_MUL = 0x03
  MATH2_CMP = 0x04
  MATH2_SAFEDIV = 0x05

class MathOper(Math1Oper, Math2Oper):
  pass

class Code:
  OK = 0x00
  OTHER_ERROR = 0x07
  MOTOR_BUSY = 0x1F
  CANCELED = 0x20
  INVALID_COMMAND = 0x2C
  INVALID_MOTOR = 0x2D
  COMMAND_IGNORED = 0x2E
  UNINVITED_POINTER = 0x2F
  READONLY_ATTRIBUTE = 0x30
  OVERFLOW = 0x31
  INVALID_CTLOP = 0x32
  INVALID_OPFLAG = 0x33
  INVALID_FUNID = 0x34
  INVALID_MATHOPER = 0x35
  USR1 = 0xFA
  USR2 = 0xFB
  USR3 = 0xFC
  USR4 = 0xFD
  USR5 = 0xFE
  UNSET = 0xFF

codes: dict[int, str] = {
  getattr(Code, name): name
  for name in dir(Code)
  if name.upper() == name}

class Condition(namedtuple('Condition', ('func', 'rhs'))):
  func: int
  rhs: int

  def __invert__(self):
    return type(self)(self.func ^ FUNC_NEGATED_FLAG, self.rhs)

  @classmethod
  def forflag(cls, name: str, negate: bool = False):
    flag: Flag = getattr(Flags, name)
    return cls(flag.attr.offset ^ (bool(negate) << FUNC_NEGATED_BIT), 1 << flag.bit)

  @classmethod
  def argeql(cls, rhs: int, negate: bool = False):
    return cls(FunId.P_CALLARG_EQL ^ (bool(negate) << FUNC_NEGATED_BIT), rhs)

  @classmethod
  def argand(cls, rhs: int, negate: bool = False):
    return cls(FunId.P_CALLARG_AND ^ (bool(negate) << FUNC_NEGATED_BIT), rhs)

  @classmethod
  def repeql(cls, rhs: int, negate: bool = False):
    return cls(FunId.EQL_RETURNCODE_RHS ^ (bool(negate) << FUNC_NEGATED_BIT), rhs)

  @classmethod
  def cmpeql(cls, rhs: int = 0, negate: bool = False):
    return cls(FunId.P_LASTCOMPARG_EQL ^ (bool(negate) << FUNC_NEGATED_BIT), rhs)

  @classmethod
  def cmplt(cls, rhs: int = 0, negate: bool = False):
    return cls(FunId.P_LASTCOMPARG_LT ^ (bool(negate) << FUNC_NEGATED_BIT), rhs)

  @classmethod
  def cmplte(cls, rhs: int = 0, negate: bool = False):
    return cls(FunId.P_LASTCOMPARG_LTE ^ (bool(negate) << FUNC_NEGATED_BIT), rhs)

  @classmethod
  def cmpgt(cls, rhs: int = 0, negate: bool = False):
    return cls(FunId.P_LASTCOMPARG_LTE ^ ((not negate) << FUNC_NEGATED_BIT), rhs)

class Script:
  def __init__(self) -> None:
    self.instructions: list[tuple[str|int, tuple[Any, ...]]|bytes] = []
    self.labels: dict[str, int] = {}
    self._size = 0
    self.compiler = Compiler()

  @property
  def size(self) -> int:
    return self._size

  def label(self, name: str) -> int:
    "Mark the NEXT instruction with this name."
    if name in self.labels:
      raise ValueError(f'Duplicate label {name!r}')
    return self.labels.setdefault(name, self.size)

  def add(self, op: str|bytes|bytearray|int, *args) -> None:
    if not args:
      if isinstance(op, int):
        if not (op == Code.OK or Code.USR1 <= op <= Code.UNSET):
          raise ValueError(f'Unknown return code {op=}')
        op = op.to_bytes(1, 'little')
      if isinstance(op, (bytes, bytearray)):
        self.instructions.append(op)
        self._size += len(op)
    else:
      self.instructions.append((op, args))
      if isinstance(op, int):
        oplen = 0
        if op == CONTROL_EXCODE:
          ctlop: CtlOp = revops[op][args[0]]
          basesize = ctlop.size
          baseop = ctlop.code
          oplen += 1
          pfxsize = ctlop.prefix_size
        else:
          basesize = revops[op].size
          baseop = op
          pfxsize = 0
        if baseop & FARPTR_OPCODE_FLAG:
          if pfxsize is None:
            raise ValueError(f'Pointers not supported for {op=} {baseop=}')
          oplen += 2 + pfxsize
        elif baseop & INDIRECT_OPCODE_FLAG:
          if pfxsize is None:
            raise ValueError(f'Pointers not supported for {op=} {baseop=}')
          oplen += 1 + pfxsize
        else:
          oplen += basesize
      else:
        if op.endswith('**'):
          baseop, modesize = op[:-2], 2
        elif op.endswith('*'):
          baseop, modesize = op[:-1], 1
        else:
          baseop, modesize = op, None
        if baseop in ctlopsmap:
          ctlop = ctlopsmap[baseop]
          if modesize:
            if ctlop.prefix_size is None:
              raise ValueError(f'Pointers not supported for {ctlop}')
            modesize += ctlop.prefix_size
          oplen = 1 + (modesize or ctlop.size)
        else:
          oplen = modesize or opsmap[baseop].size
      self._size += 1 + oplen

  def compile(self, resolver: Callable|None = None, **kw) -> bytes:
    "Resolve labels and pack bytes."
    return self.compiler.compile(self, resolver=resolver or self.resolve, **kw)

  def pprintf(self):
    return self.compiler.pprintf()

  def pprint(self):
    self.compiler.pprint()

  def resolve(self, arg: str|int|float) -> int|float:
    if isinstance(arg, str):
      if arg.startswith(':') and arg in ctlopsmap:
        return ctlopsmap[arg].code
      # Resolve Label strings to absolute byte offsets
      return self.labels[arg]
    return arg

  def if_flag(self, name: str, negate: bool = False):
    return If(self, Condition.forflag(name, negate=negate))

  def if_argeql(self, rhs: int, negate: bool = False):
    return If(self, Condition.argeql(rhs, negate=negate))

  def if_argand(self, rhs: int, negate: bool = False):
    return If(self, Condition.argand(rhs, negate=negate))

  def if_repeql(self, rhs: int, negate: bool = False):
    return If(self, Condition.repeql(rhs, negate=negate))

  def while_flag(self, name: str, negate: bool = False):
    return While(self, Condition.forflag(name, negate=negate))

  def while_vareql(self, varid: int, threshold: int, negate: bool = False):
    return WhileCompare(self, Condition.cmpeql(negate=negate), varid, threshold)

  def while_varlt(self, varid: int, threshold: int, negate: bool = False):
    return WhileCompare(self, Condition.cmplt(negate=negate), varid, threshold)

  def while_varlte(self, varid: int, threshold: int, negate: bool = False):
    return WhileCompare(self, Condition.cmplte(negate=negate), varid, threshold)

  def while_vargt(self, varid: int, threshold: int, negate: bool = False):
    return WhileCompare(self, Condition.cmpgt(negate=negate), varid, threshold)

class If:
  def __init__(self, script: Script, condition: Condition) -> None:
    self.script = script
    self.condition = condition
    self.id = len(script.instructions)
    self.else_label = f'if_else_{self.id}'
    self.exit_label = f'if_exit_{self.id}'
    self._has_run_else = False

  def __enter__(self):
    # If NOT condition, jump to the ELSE marker
    self.script.add(*CtlOps.cond_jump[:2], *~self.condition, Code.UNSET, self.else_label)
    return self

  def else_(self):
    "Transition point between True and False branches."
    self._has_run_else = True
    # True block is finished; jump over the upcoming Else block
    self.script.add(*CtlOps.jump[:2], Code.UNSET, self.exit_label)
    # Mark where the Else block starts
    self.script.label(self.else_label)

  def __exit__(self, exc_type, exc_val, exc_tb):
    # If we didn't have an else block, the jump-target is just 'here'
    if not self._has_run_else:
      self.script.label(self.else_label)
    self.script.label(self.exit_label)

class While:
  def __init__(self, script: Script, condition: Condition) -> None:
    self.script = script
    self.condition = condition
    self.id = len(script.instructions)
    self.start_label = f'loop_s_{self.id}'
    self.exit_label = f'loop_e_{self.id}'

  def __enter__(self):
    self.script.label(self.start_label)
    # 'If NOT (condition), Jump to Exit'
    self.script.add(*CtlOps.cond_jump[:2], *~self.condition, Code.UNSET, self.exit_label)
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    self.script.add(*CtlOps.jump[:2], Code.UNSET, self.start_label)
    self.script.label(self.exit_label)

class WhileCompare:
  def __init__(self, script: Script, condition: Condition, varid: int, threshold: int) -> None:
    self.script = script
    self.condition = condition
    self.varid = varid
    self.threshold = threshold
    self.id = len(script.instructions)
    self.start_label = f'w_s_{self.id}'
    self.exit_label = f'w_e_{self.id}'

  def __enter__(self):
    self.script.label(self.start_label)
    # 1. Update compArg: (Var - Threshold)
    self.script.add(*CtlOps.var_math2[:2], self.varid, Math2Oper.MATH2_CMP, self.threshold)
    # 2. If NOT (Condition), Jump to Exit
    self.script.add(*CtlOps.cond_jump[:2], *~self.condition, Code.UNSET, self.exit_label)
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    # 3. Jump back and close
    self.script.add(*CtlOps.jump[:2], Code.UNSET, self.start_label)
    self.script.label(self.exit_label)

class CompileError(Exception):
  pass

class Compiler:
  def __init__(self, resolver: Callable|None = None):
    self.resolver = resolver
    self.executable = None

  def compile(self, script: Script, resolver: Callable|None = None):
    self.resolver = resolver or self.resolver
    self.script = script
    self.executable = bytearray()
    starts = array.array('B')
    ends = array.array('B')
    self.opnames = []
    
    for i, instruction in enumerate(script.instructions):
      try:
        context = CompilerInstructionContext(self, instruction)
        context.run()
        starts.append(len(self.executable))
        self.executable.extend(context.buf)
        ends.append(len(self.executable))
        self.opnames.append(context.opname)
        if len(self.executable) > SCRIPT_PAGE_SIZE:
          raise CompileError(f'{SCRIPT_PAGE_SIZE=} exceeded: {len(self.executable)}')
      except Exception as err:
        print(f'{err!r} {i=} {instruction=}')
        raise
    mem = memoryview(self.executable)
    self.lines = tuple(
      mem[start:end] for start, end in zip(starts, ends))
    return bytes(self.executable)

  def _pprintf_line(self, i: int):
    line = self.lines[i]
    opname = self.opnames[i] or ''
    linestr = ' '.join(f'{x:02X}' for x in line)
    return f'{linestr:<26s} | {opname}'

  def pprintf(self):
    return '\n'.join(self._pprintf_line(i) for i in range(len(self.lines)))

  def pprint(self):
    print(self.pprintf())

class CompilerInstructionContext:
  def __init__(self, compiler: Compiler, instruction: tuple[str|int, tuple[Any, ...]]|bytes):
    self.compiler = compiler
    self.instruction = instruction
    self.resolver = compiler.resolver
    self.opname = None

  def run(self):
    self.buf = bytearray()
    if isinstance(self.instruction, (bytes, bytearray)):
      self.buf.extend(self.instruction)
      return
    self.op, self.args = self.instruction
    if self.resolver:
      self.args = tuple(map(self.resolver, self.args))
    if isinstance(self.op, int):
      self.handle_int_op()
    else:
      self.handle_str_op()
    self.buf.append(self.opcode)
    self.buf.extend(struct.pack(b'<'+self.fmt, *self.args))

  def handle_int_op(self):
    if self.op == CONTROL_EXCODE:
      # For Control, args[0] is the sub-opcode (e.g., 0x01, 0x41, 0x81)
      sub_op, *self.args = self.args
      
      # Mask out flags to find the base CtlOp definition
      base_sub_op = sub_op & ~(INDIRECT_OPCODE_FLAG | FARPTR_OPCODE_FLAG)
      ctlop: CtlOp = revops[self.op][base_sub_op]
      
      # Prepend 0xC0 to the buffer immediately
      self.buf.append(self.op) 
      self.opcode = sub_op
      self.opname = ctlop.name

      # Determine the format based on bitmask
      if sub_op & FARPTR_OPCODE_FLAG:
        # RegIdx (B) + Page (B) + Ptr (B)
        if ctlop.prefix_size is None:
          raise CompileError(f'Addressing modes not supported for {ctlop.name}')
        self.fmt = b'B' * ctlop.prefix_size + b'BB'
        self.opname += '**'
      elif sub_op & INDIRECT_OPCODE_FLAG:
        # RegIdx (B) + Ptr (B)
        if ctlop.prefix_size is None:
          raise CompileError(f'Addressing modes not supported for {ctlop.name}')
        self.fmt = b'B' * ctlop.prefix_size + b'B'
        self.opname += '*'
      else:
        # Literal format defined in CtlOp (e.g., 'Bl')
        self.fmt = ctlop.fmt
        
    elif self.op & FARPTR_OPCODE_FLAG:
      self.fmt = b'BB'
      self.opcode = self.op
    elif self.op & INDIRECT_OPCODE_FLAG:
      self.fmt = b'B'
      self.opcode = self.op
    else:
      dop = revops[self.op]
      self.fmt = dop.fmt
      self.opcode = self.op
      self.opname = dop.name

    if not self.opname:
      directop = self.opcode & ~(FARPTR_OPCODE_FLAG | INDIRECT_OPCODE_FLAG)
      if directop in revops:
        dop = revops[directop]
        if isinstance(dop, Attribute):
          self.opname = dop.name
          if self.opcode & FARPTR_OPCODE_FLAG:
            self.opname += '**'
          elif self.opcode & INDIRECT_OPCODE_FLAG:
            self.opname += '*'

  def handle_str_op(self):
    # Handling named opcodes (e.g. 'move*', ':set_var')
    if self.op.endswith('**'):
      base_name, mask, ptr_fmt = self.op[:-2], FARPTR_OPCODE_FLAG, b'BB'
    elif self.op.endswith('*'):
      base_name, mask, ptr_fmt = self.op[:-1], INDIRECT_OPCODE_FLAG, b'B'
    else:
      base_name, mask, ptr_fmt = self.op, 0, None

    if base_name in ctlopsmap:
      ctlop = ctlopsmap[base_name]
      self.buf.append(ctlop.esc) # Prepend 0xC0
      self.opcode = ctlop.code | mask
      self.opname = ctlop.name
      
      if ptr_fmt:
        if ctlop.prefix_size is None:
          raise CompileError(f'Addressing modes not supported for {ctlop.name}')
        # Pointer mode: RegIdx (B) + Pointer (B or BB)
        self.fmt = b'B' * ctlop.prefix_size + ptr_fmt
      else:
        # Literal mode: Use default format (e.g. 'Bl')
        self.fmt = ctlop.fmt
    else:
      # Standard Attribute (move, speed, etc)
      attr = opsmap[base_name]
      self.opcode = attr.offset | mask
      self.opname = attr.name
      self.fmt = ptr_fmt or attr.fmt
    if mask == FARPTR_OPCODE_FLAG:
      self.opname += '**'
    elif mask == INDIRECT_OPCODE_FLAG:
      self.opname += '*'

def _cpp_x40map(name: str, arr: list[int]) -> str:
    numstrs = [f'0x{x:02X},' for x in arr]
    chunks = [numstrs[i:i+8] for i in range(0, 0x40, 8)]
    decl = f'static const uint8_t {name}[0x40] = {{%s}};'
    body: str = '\n'.join((f'  {" ".join(chunk)}' for chunk in chunks))
    return decl % body.join(2*'\n')