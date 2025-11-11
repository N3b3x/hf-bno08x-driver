---
layout: default
title: "Doxygen Extensions"
nav_order: 1
parent: "Documentation"
description: "Advanced Doxygen documentation with automatic dark mode, interactive features, and modern styling"
has_children: true
has_toc: true
---

# 📖 Doxygen Extensions

This section contains advanced Doxygen documentation features and extensions.

## 🌟 Features

Our Doxygen documentation includes several modern enhancements:

- **🌓 Automatic Dark Mode** - Detects system preference and switches automatically
- **📋 Copy Buttons** - Hover over code blocks to copy them to clipboard
- **🔗 Paragraph Links** - Click the ¶ symbol to get direct links to sections
- **📑 Interactive TOC** - Dynamic table of contents with progress highlighting
- **📑 Tabs** - Organize content in tabbed interfaces
- **📱 Responsive Design** - Works perfectly on desktop and mobile devices

## 📁 Structure

- **[Configuration](doxygen-extensions/)** - Doxygen configuration and setup files
- **[Submodule](doxygen-extensions/doxygen-awesome-css/)** - Doxygen-awesome-css theme files

## 🔧 Technical Details

The documentation system uses:

- **Doxygen** - For generating API documentation from source code
- **doxygen-awesome-css** - Modern CSS theme with dark mode support
- **Custom Extensions** - JavaScript enhancements for better user experience
- **Automatic Detection** - System preference detection for theme switching

## 📖 Usage

To generate the documentation locally:

```bash
doxygen _config/Doxyfile
```

The generated documentation will be available in the `docs/doxygen/` directory.
