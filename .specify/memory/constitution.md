<!-- SYNC IMPACT REPORT
Version change: N/A (initial version) → 1.0.0
Added sections: All principles and sections based on user input
Removed sections: None
Modified principles: N/A (first version)
Templates requiring updates:
- .specify/templates/plan-template.md ✅ updated
- .specify/templates/spec-template.md ✅ updated
- .specify/templates/tasks-template.md ✅ updated
- .specify/templates/commands/*.md ✅ reviewed
- README.md ✅ N/A (file does not exist)
Follow-up TODOs: RATIFICATION_DATE to be set when constitution is formally adopted
-->
# AI-Driven Book Creation using Docusaurus, GitHub Pages, Spec-Kit Plus, and Claude Code Constitution

## Core Principles

### I. Beginner-Friendly Clarity
Simple, clear writing that beginners can easily understand. All content must use accessible language, avoid jargon without explanation, and ensure concepts are explained step-by-step. Content must be self-contained and comprehensible without requiring external references for basic understanding.

### II. Engaging Pedagogy
Maintain a friendly, engaging, step-by-step teaching style. Each section should connect logically to the previous one while maintaining an encouraging tone. Content must include practical examples, hands-on activities, and clear learning objectives that guide the student through progressive skill building.

### III. Specification-First Development
Every major chapter or section must have a Speckit specification BEFORE writing content. No content creation occurs without an approved specification that defines scope, learning objectives, and acceptance criteria. This ensures consistency and prevents scope creep during development.

### IV. Progressive Learning Structure
Teach from basic to advanced concepts smoothly, ensuring each concept builds naturally on previous knowledge. Concepts must be introduced in logical sequence with adequate scaffolding. Students should be able to follow the progression without encountering gaps in prerequisite knowledge.

### V. Consistent Quality Standards
Maintain consistent structure, tone, formatting, and style across the whole book. All chapters follow identical structural templates, use uniform terminology, and maintain consistent formatting standards. This creates a seamless learning experience across all content.

### VI. Accuracy and Citations
Provide accurate information with reliable sources when needed. All claims must be fact-checked and sourced appropriately. Technical information must be verified for correctness, and external resources must come from reputable, authoritative sources.

## Key Standards

All content must be Docusaurus-compatible Markdown/MDX. All code examples must be correct, tested, and easy to run. Each chapter must include: a clear title, learning goals, simple explanation, practical examples, step-by-step instructions, a summary, and optional exercises or mini-projects. Images must include alt-text and be optimized for web. Writing tone: helpful teacher, easy words, enjoyable reading. Zero plagiarism — all content must be original or properly cited.

Structure requirements: At least 10 major chapters. Must include introduction, conclusion, glossary, and resources page. Chapters must be beginner-friendly but progressively advance in difficulty. Sidebar and folder structure must follow Docusaurus conventions. Each chapter should stand independently but still connect to the full book flow.

## Workflow Rules

No chapter is written until its Speckit specification is approved. All tasks must be converted into clear, structured specs before content creation. Claude Code must always follow the constitution. All generated files must be commit-ready and Git-safe. Book must compile cleanly using: `npm run build`. Language: English. Output format: Docusaurus Markdown/MDX only. Deployment: GitHub Pages. No broken links, no empty sections, no formatting errors. Avoid overly complex or academic language—keep it simple.

## Governance

This constitution governs all aspects of the book creation process. All contributors must adhere to these principles and standards. Any deviation from these principles requires explicit approval and documentation of the exception. Changes to this constitution follow a formal amendment process with stakeholder review and approval. All deliverables must be validated against these principles before acceptance.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 (initial version) | **Last Amended**: 2025-12-16