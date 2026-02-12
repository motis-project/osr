This subproject is a debug UI for the OSR map matching module.

- Use pnpm, not npm
- Use TailwindCSS for styling
- Use Shadcn UI for components (e.g. `pnpm dlx shadcn@latest add tabs` to install a new component)
- Use Maplibre GL JS for the map
- We only support dark mode

The JSON files read by this UI are created by the C++ code in osr/src/map_matching_debug.cc.
Whenever changes to the file format are made, make sure the C++ code generating the files and the TypeScript code reading the files are kept in sync.