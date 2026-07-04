# Google C++ Style Guide — Condensed

Operative digest of [google-cppguide.html](google-cppguide.html) (the vendored
full guide) with this project's **PROJECT OVERRIDE** banners applied inline.
Where this file and the full guide disagree, the override wins. Override
rationale and full detail: `.claude/rules/naming-and-style.md` and
`.claude/rules/coding-standards.md`.

Meta-rules: optimize for the reader, not the writer. Be consistent with
surrounding code. Avoid clever, surprising, or tricky constructs — when in
doubt, write the boring version.

## C++ version and extensions

- Target **C++20**; do not use C++23 features.
- No nonstandard compiler extensions (`__attribute__`, `#pragma`, inline asm,
  VLAs/`alloca`, `a?:b`, `__builtin_*`, compound statement expressions) except
  through a designated project-wide portability wrapper.

## Naming — PROJECT OVERRIDE (CamelCase; replaces Google's case rules)

| Entity | Style | Example |
|---|---|---|
| Types (class, struct, enum, alias, concept, type template param) | UpperCamelCase | `UrlTable`, `HTTPServer` |
| Namespaces | UpperCamelCase | `namespace Hal` |
| Functions and methods — **never PascalCase** | lowerCamelCase | `setVelocity()`, `tick()`, `addTableEntry()` |
| Variables and parameters | lowerCamelCase | `tableName`, `leftObs`, `httpRequest` |
| Class data members | lowerCamelCase + trailing `_` | `lastPosition_` |
| Struct data members | lowerCamelCase, no trailing `_` | `numEntries` |
| Constants (const/constexpr with static storage) | `k` + UpperCamelCase | `kDaysInAWeek`, `kTableVersion` |
| Enumerators (scoped and unscoped) | like constants, never MACRO_STYLE | `kOutOfMemory` |
| Macros (avoid) | project-prefixed ALL_CAPS | `MYPROJECT_ROUND` |
| Files | lowercase snake_case per the guide; follow the tree's existing convention where it differs (e.g. `source/`'s `CommandProcessor.cpp`) | `motor_controller.h` |

- Acronyms follow the case of their position: `HTTPServer` (type),
  `httpRequest` (variable/function).
- Mathematical subscripts keep their underscore: `v_x`, `x_b` — notation, not
  word separation.
- **No units in any identifier** (field, function, parameter). Name the
  quantity (`speed`, `velocity`, `deadline`); the unit goes in a leading
  bracketed tag as the first token of the trailing comment:
  `float speed;  // [mm/s]`. `speed` = magnitude, `velocity` = directed; a
  twist has components (`v_x`, `v_y`, `omega`); positions are `x`, `y`.
- Names must make purpose clear to a new reader; descriptiveness proportional
  to scope of visibility. No obscure abbreviations; never delete letters
  within a word (`cstmrId` bad). `i` for a loop index and `T` for a template
  parameter are fine.
- Generated code (`source/messages/*`, `msg::`) is exempt. Legacy
  (`source_old`) stays as-is until touched.
- **When editing code that predates these rules, bring the touched code into
  conformance** rather than matching the old style (project rule; replaces the
  guide's be-consistent-with-legacy leniency for this repo's own code).

## Header files

- Every `.cc` has a matching `.h` (exceptions: tests, `main()`-only files).
- Headers are self-contained: header guards, and they include everything they
  use. Include-fragments that aren't self-contained end in `.inc` (rare).
- Guard format: `<PROJECT>_<PATH>_<FILE>_H_`.
- Include what you use; never rely on transitive includes.
- Avoid forward declarations; include the header instead. Never
  forward-declare entities you don't own, and never from `std::`.
- Define a function in a header only if it's ~10 lines or fewer; longer bodies
  go in the `.cc`, or (templates/constexpr) in an internal section below the
  public API. Header definitions must be ODR-safe (`inline` or implicitly so).
- Include order, groups separated by one blank line, alphabetical within each:
  1. the related header (`foo.h` first in `foo.cc`),
  2. C/system headers in `<...>` with `.h`,
  3. C++ standard library headers,
  4. other libraries' headers,
  5. this project's headers.
- Project headers by full path from the source root (`"base/logging.h"`);
  never `.` or `..`. Angle brackets only where the library requires them.

## Scoping

- Place code in a namespace named after the project/path; the namespace wraps
  the whole file after includes. Contents are not indented. Terminate with
  `}  // namespace Foo`.
- Never `using namespace foo;`. Never inline namespaces. No namespace aliases
  at namespace scope in headers (internal-marked namespaces excepted). Never
  declare anything in `namespace std`.
- Give `.cc`-only helpers internal linkage (unnamed namespace or `static`);
  never do this in headers.
- Prefer nonmember functions in a namespace over global functions; do not
  create a class just to group static members.
- Declare locals in the narrowest scope, initialized at declaration
  (`int i = f();`, never declare-then-assign), close to first use. Exception:
  an expensive ctor/dtor object used in a loop may be hoisted out.
- Objects with static storage duration must be **trivially destructible**: no
  global `std::string`, containers, or smart pointers. `constexpr` values,
  fundamental types, and arrays of them are fine. Mark constant-initialized
  statics `constexpr`/`constinit`. Dynamic initialization of nonlocal statics
  is forbidden unless no code depends on its ordering. Function-local static
  dynamic init is fine. Escape hatch: `static const auto& x = *new T(...);`
  (never deleted).
- `thread_local` at class/namespace scope must be `constinit`; prefer
  function-local `thread_local`; prefer `thread_local` over other TLS.

## Classes

- Constructors: never call virtual functions; avoid work that can fail (no
  exceptions to report it) — use a factory function or `init()` method when
  initialization can fail.
- Mark single-argument constructors and conversion operators `explicit`
  (copy/move constructors and `std::initializer_list` constructors exempt).
- Make copyability/movability explicit in the public API: declare/`= default`/
  `= delete` the copy and move operations. Provide move ops for a copyable
  type only when meaningfully faster than copying. If you declare one of a
  ctor/assignment pair, declare the other.
- Prevent slicing: make base classes abstract (protected ctor/dtor or a pure
  virtual); avoid deriving from concrete classes.
- `struct` = passive data carrier: all fields public, no invariants between
  fields. Anything with behavior or invariants is a `class`. In doubt: class.
- Prefer a struct over `std::pair`/`std::tuple` whenever elements can be
  meaningfully named.
- Prefer composition over inheritance; inherit only for genuine "is-a". All
  inheritance is `public` (want private inheritance → use a member). Multiple
  implementation inheritance strongly discouraged. `final` OK on classes not
  meant as bases.
- Annotate every override with exactly one of `override`/`final`; do not
  repeat `virtual` on overrides.
- Data members are `private` (constants excepted; GoogleTest fixtures in a
  `.cc` may use protected). Limit `protected` to member functions.
- Operator overloading: only with obvious, conventional meaning. Define
  operators in the same header/`.cc`/namespace as their type; non-modifying
  binary operators as non-member functions. Define `operator==` for
  value-comparable types; `operator<=>` only with one obvious ordering. Never
  overload `&&`, `||`, `,`, unary `&`, or `operator""` — no user-defined
  literals, not even the standard library's.
- Declaration order: `public:`, `protected:`, `private:`; within each section:
  types/aliases, (struct-only data,) static constants, factory functions,
  constructors/assignment, destructor, other functions, data members. No
  large inline method bodies in the class definition.
- `friend` allowed within reason, usually defined in the same file.

## Functions

- Prefer return values over output parameters; prefer returning by value.
  Inputs before outputs in the parameter list. Non-optional inputs: value or
  const reference. Required outputs: reference. Optional by-value inputs:
  `std::optional`. Optional outputs/in-outs: non-const pointer. Don't design
  functions whose reference parameters must outlive the call — use a pointer
  and document lifetime instead.
- Keep functions small and focused; reconsider anything over ~40 lines.
- Overload only when a reader at the call site doesn't need to know which
  overload is chosen; document the overload set with one umbrella comment.
- Default arguments: allowed on non-virtual functions when the default is
  always the same value; **banned on virtual functions**; prefer overloads
  when in doubt.
- Trailing return type (`auto f(...) -> T`) only where required (lambdas) or
  clearly more readable (complex template return types).

## Ownership and memory

- Dynamically allocated objects get a single fixed owner; transfer ownership
  with `std::unique_ptr` (`std::unique_ptr<Foo> fooFactory();`). Never
  `std::auto_ptr`.
- Shared ownership only with a strong reason (e.g. avoiding expensive copies
  of an immutable payload), via `std::shared_ptr<const T>`.
- Rvalue references only for: move constructor/assignment, `&&`-qualified
  methods that consume `*this`, forwarding references with `std::forward`,
  and `const Foo&`/`Foo&&` overload pairs. To consume a parameter, prefer
  pass-by-value.

## Forbidden or restricted

- **No C++ exceptions**, including `std::exception_ptr` and
  `std::nested_exception`. `noexcept`: use where useful and correct
  (unconditional when exceptions are disabled; assume it pays on move ctors).
- No RTTI (`typeid`/`dynamic_cast`) in production logic — use virtual
  dispatch or Visitor. Free use in unit tests. `dynamic_cast` only when the
  logic guarantees the derived type.
- **No C-style casts** (`(int)x`). Use, in order of preference: brace init for
  arithmetic conversions (`int64_t{x}` — refuses to compile on data loss);
  function-style for class types (`std::string(someCord)`); `static_cast` for
  pointer up/down-casts you can guarantee; `const_cast`; `reinterpret_cast`
  (rare, know the aliasing rules); `std::bit_cast` for type-punning.
- No C++20 modules. Coroutines only via lead-approved libraries — never roll
  your own promise/awaitable types.
- Disallowed std library: `<ratio>`, `<cfenv>`/`<fenv.h>`, `<filesystem>`.
- Macros: avoid, especially in headers, and never to define API pieces. If
  unavoidable: not in a `.h`; `#define` just before use, `#undef` right
  after; unique project-prefixed name; avoid `##` name generation.
- Avoid complicated template metaprogramming (recursive instantiation, type
  lists, SFINAE tricks = too far); isolate and document any that's justified.

## Language usage

- `++i`, not `i++`, unless postfix semantics are needed.
- `const` in APIs wherever meaningful: reference/pointer params a function
  won't modify are `const T&`/`const T*`; methods that don't change logical
  state are `const` (and all const ops must be safe to call concurrently, or
  document the class thread-unsafe). Don't `const`-qualify by-value params in
  declarations. `const int* foo` word order preferred. Local-variable `const`
  is optional.
- `constexpr` for true constants and the functions supporting them;
  `constinit` to enforce constant initialization; `consteval` for
  compile-time-only functions. Don't contort code to be constexpr, and don't
  use them to force inlining.
- Integers: `int` for known-small values (assume 32 bits, no more); otherwise
  fixed-width `<stdint.h>` types (`int16_t`, `int64_t`, …). Anything that can
  reach 2³¹ gets a 64-bit type. Never `short`/`long`/`long long`. **No
  unsigned types just to say "non-negative"** — unsigned only for bitfields
  or intentional modular arithmetic. `size_t`/`ptrdiff_t` OK where apt.
- Floating point: `float`/`double` only; never `long double`. FP literals
  always carry a radix point with digits on both sides (`1.0f`, `1248.0e6`).
- Portability: type-safe formatting (not raw `printf`); serialize structured
  data rather than copying in-memory representations; `uintptr_t` for
  addresses-as-integers; braced init for 64-bit constants
  (`uint64_t mask{uint64_t{3} << 48};`).
- `nullptr` for pointers; `'\0'` for chars — never literal `0`.
- `sizeof(varName)` over `sizeof(Type)` when a variable exists.
- Type deduction (`auto`) only when it makes the code **clearer or safer** for
  a reader outside the project — obvious inits (`auto w =
  std::make_unique<Widget>(...)`, iterators), never merely to save typing.
  Deduced return types: only tiny, narrow-scope functions — almost never in
  public headers. No `auto` parameters outside lambdas — use named template
  parameters. Structured bindings encouraged for map/pair elements; annotate
  non-obvious binding names `auto [/*field=*/boundName, ...]`.
- CTAD only with templates that ship at least one explicit deduction guide
  (all of `std` presumed opted in).
- Designated initializers in C++20-compliant form only: fields in declaration
  order (`Point p = {.x = 1.0, .y = 2.0};`).
- Lambdas: prefer explicit captures whenever the lambda may escape the current
  scope; `[&]` only when the lambda is obviously outlived by its captures;
  `[=]` only for short lambdas binding a few obvious variables, and never
  implicitly capturing `this` (capture `this` explicitly). Don't use init
  captures to invent new names — declare a variable, then capture it.
- Concepts sparingly — only where pre-C++20 code would have used templates.
  Prefer standard concepts over hand-rolled traits; `requires(Concept<T>)`
  syntax, not `template <Concept T>`; no concepts the compiler can't enforce;
  new concept definitions rare and library-internal.
- Aliases: `using`, not `typedef`. Public aliases only when clients are meant
  to use them, with documented intent. Convenience aliases belong in `.cc`
  files, function bodies, or private sections — never a header's public API.
- `switch`: if not exhaustively switching an enum, always provide `default`
  (treat unreachable defaults as fatal errors). Annotate intentional
  fall-through `[[fallthrough]];` (consecutive empty labels exempt).
- Streams only for ad-hoc, local, developer-facing I/O; avoid stateful stream
  API and manipulators. Overload `<<` only for value types, printing the
  user-visible value — debug internals go in a named method instead.
- Third-party/library preference order: the codebase's established choice,
  then Abseil, then the C++ standard library, then first-party, then
  third-party.
- Inclusive language in code and comments: no "master/slave",
  "blacklist/whitelist", etc.; gender-neutral "they" for people, "it" for
  software.

## Comments

- The best code self-documents; comment what names and types cannot say.
  `//` style; consistent. Proper spelling, punctuation, grammar.
- Function declarations: a preceding comment describing use — verb-phrase form
  ("Opens the file"), covering inputs/outputs, ownership/lifetime of pointer
  args, nullability, what happens to in/out state, perf caveats. Omit only
  for the truly obvious (trivial accessors). Definitions: explain the tricky
  "how", don't restate the declaration comment.
- Every non-obvious class gets a comment: purpose, usage, thread-safety
  assumptions, ideally a small example.
- Data members: comment invariants and sentinel values the type/name can't
  express (`int numTotalEntries_;  // -1 means not yet known`). Units always
  in the leading bracketed tag: `float tgtSpeed[kWheelCount] = {};  // [mm/s]
  all-wheel speed targets`.
- Global variables: comment what and why global.
- Don't state the obvious; comment *why*, or refactor until self-describing.
- Nonobvious call-site arguments: prefer named constants, enums over bools,
  option structs, or explanatory variables; last resort
  `/*paramName=*/value`.
- `// TODO: bug/link - actionable description` with a specific date or event
  if deferred.
- New files: license boilerplate; no author lines. File-level comment only
  when a file holds several related abstractions (don't duplicate the `.h`
  comment in the `.cc`).

## Formatting

- **80 columns max** (exempt: unsplittable comments/strings/URLs, includes,
  header guards, using-declarations). **2-space indent, spaces only, never
  tabs.** No trailing whitespace.
- Functions: return type on the same line as the name; open paren tight
  against the name; open brace at the end of the same line with one space
  before it. Wrap params aligned with the first param, or all on new lines at
  4-space indent. Unused params: omit the name only when obvious, else
  comment it out — `void Circle::rotate(double /*radians*/) {}`. Attributes
  (`[[nodiscard]]`) go before the return type.
- Conditionals/loops: space after keyword, no padding inside parens, brace on
  the same line — `if (condition) {`. Brace every controlled statement; a
  brief single statement may sit braceless on one line
  (`if (x == kFoo) return new Foo();`) or the next line, but never with
  `else`/`do-while`, and never spanning conditions. `} else if (...) {` /
  `} else {` on the closing-brace line. Empty loop body: `{}` or `continue;`,
  never a bare `;`.
- `switch`: cases indented 2, bodies 4; case braces optional.
- Pointers/refs: asterisk/ampersand binds left, to the type — `char* c;`,
  `const std::string& str;`, `std::vector<char*>`. No multi-variable
  declarations containing `*` or `&`.
- Wrapped boolean expressions keep the operator at end of line (`&&` at EOL).
- No parentheses around simple `return` expressions.
- `=`, `()`, and `{}` initialization all acceptable. Careful:
  `std::vector<int> v(100, 1)` (100 ones) vs `{100, 1}` (two elements);
  braces prevent narrowing (`int pi{3.14};` won't compile).
- Braced init lists format exactly like function calls.
- Preprocessor directives start at column 0, even inside indented code.
- Class format: `public:`/`protected:`/`private:` indented **one** space, in
  that order, preceded by a blank line (except the first), no blank line
  after:

  ```cpp
  class MyClass : public OtherClass {
   public:
    MyClass();
    explicit MyClass(int var);

    void someFunction();
    void setSomeVar(int var) { someVar_ = var; }
    int someVar() const { return someVar_; }

   private:
    int someVar_;
  };
  ```

- Constructor initializer lists: on one line, or wrap before the `:` with a
  4-space indent, one member per line, aligned.
- Namespace contents are not indented.
- Horizontal whitespace: two spaces before end-of-line comments; spaces around
  assignment and (usually) binary operators (droppable around factors:
  `w*x + y/z`); none after unary operators; none inside angle brackets or
  parens; space around the `:` in inheritance, init lists, and range-for.
- Vertical whitespace sparingly: blank lines as paragraph breaks only; none at
  the start or end of a block.
