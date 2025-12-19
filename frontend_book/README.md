# Physical AI & Humanoid Robotics Book

This is the documentation website for the Physical AI & Humanoid Robotics educational book, built with Docusaurus.

## Prerequisites

- Node.js version 18.0 or above
- npm or yarn package manager

## Installation

```bash
# Navigate to the frontend_book directory
cd frontend_book

# Install dependencies
npm install
```

## Local Development

```bash
# Start the development server
npm run start

# The site will be accessible at http://localhost:3000
```

## Build for Production

```bash
# Build the static files
npm run build

# Serve the built files locally for testing
npm run serve
```

## Project Structure

```
frontend_book/
├── docs/                 # Documentation files
│   ├── intro.md          # Introduction page
│   ├── module-1-ros/     # Module 1: The Robotic Nervous System
│   ├── module-2-digital-twin/ # Module 2: The Digital Twin
│   ├── module-3-ai-brain/     # Module 3: The AI-Robot Brain
│   ├── module-4-vla/         # Module 4: Vision-Language-Action
│   └── capstone/            # Capstone Project
├── src/                  # Custom React components
├── static/              # Static assets
├── docusaurus.config.ts # Docusaurus configuration
├── sidebars.ts          # Navigation sidebar configuration
└── package.json         # Project dependencies and scripts
```

## Documentation Modules

1. **Module 1: The Robotic Nervous System (ROS 2)** - Covers ROS 2 architecture and communication
2. **Module 2: The Digital Twin (Gazebo & Unity)** - Simulation and digital twin concepts
3. **Module 3: The AI-Robot Brain (NVIDIA Isaac™)** - AI perception and navigation
4. **Module 4: Vision-Language-Action (VLA)** - Natural language interaction with robots
5. **Capstone Project** - Complete autonomous humanoid robot integration

## Contributing

To add new content:

1. Create new markdown files in the appropriate module directory
2. Update `sidebars.ts` to include the new page in the navigation
3. Use Docusaurus markdown features for enhanced documentation

## Deployment

The site can be deployed to various platforms:

- GitHub Pages
- Netlify
- Vercel
- Any static hosting service

For GitHub Pages deployment, use:
```bash
npm run deploy
```
