## Behavior

**Think before coding.** Surface assumptions and tradeoffs before writing. Don't hide confusion — ask.

**Simplicity first.** Minimum code that solves the problem. Nothing speculative, no unrequested features, no unnecessary abstractions.

**Surgical changes.** Touch only what you must. Match existing style. Don't refactor or clean up unrelated code.

**Goal-driven.** Define a brief plan with a clear success condition before starting. Verify it works before declaring done.

## graphify

This project has a graphify knowledge graph at graphify-out/.

Rules:
- Before answering architecture or codebase questions, read graphify-out/GRAPH_REPORT.md for god nodes and community structure
- If graphify-out/wiki/index.md exists, navigate it instead of reading raw files
- After modifying code files in this session, run `graphify update .` to keep the graph current (AST-only, no API cost)
