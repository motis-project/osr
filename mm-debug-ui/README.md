# Map Matching Debug UI

This is a debug UI for the map matching algorithm in OSR.

Usage:

```bash
pnpm run dev
```

- Use the `debug_path` parameter of the `map_match` function in the C++ code to generate a JSON file that can be loaded into the debug UI.
- Open the debug UI
- Drag & drop the generated JSON file into the debug UI

WARNING: This debug UI is fully vibe-coded without human code review.
