# Contributing to StreamDeploy Containers

Thank you for your interest in contributing to StreamDeploy Containers! This document provides guidelines for contributing new containers, bug fixes, and improvements to the project.

## Table of Contents
- [Getting Started](#getting-started)
- [Container Structure](#container-structure)
- [Adding a New Container](#adding-a-new-container)
- [Testing Your Container](#testing-your-container)
- [Submitting a Pull Request](#submitting-a-pull-request)
- [Review Process](#review-process)
- [Code of Conduct](#code-of-conduct)

## Getting Started

1. **Fork the repository** on GitHub
2. **Clone your fork** locally:
   ```bash
   git clone https://github.com/YOUR_USERNAME/streamdeploy-containers.git
   cd streamdeploy-containers
   ```
3. **Install dependencies**:
   ```bash
   npm install
   ```
4. **Create a new branch** for your contribution:
   ```bash
   git checkout -b feature/your-container-name
   ```

## Container Structure

Each container must follow this exact directory structure:

```
containers/
└── your-container-slug/
    ├── Dockerfile          # Container build instructions
    ├── entrypoint.sh      # Container entry point script
    ├── meta.json          # Container metadata
    └── README.md          # Container documentation
```

### Required Files

#### 1. `meta.json`
Contains container metadata following our schema:

```json
{
  "name": "Your Container Name",
  "tagline": "Brief description of what this container does",
  "tags": ["tag1", "tag2", "tag3"],
  "architectures": ["amd64", "arm64"],
  "primaryHw": "generic"
}
```

**Fields:**
- `name`: Human-readable container name
- `tagline`: Brief description (max 100 characters)
- `tags`: Array of relevant tags for categorization
- `architectures`: Supported CPU architectures (`amd64`, `arm64`)
- `primaryHw`: Primary hardware target (`generic`, `raspberry-pi`, `jetson`, `coral`, etc.)

#### 2. `Dockerfile`
Standard Docker build file with:
- Clear base image selection
- Proper dependency installation
- Security best practices
- Multi-architecture support (when applicable)

#### 3. `entrypoint.sh`
Executable shell script that:
- Has proper shebang (`#!/bin/bash` or `#!/bin/sh`)
- Is executable (`chmod +x entrypoint.sh`)
- Handles environment variables appropriately
- Provides clear error messages

#### 4. `README.md`
Comprehensive documentation including:
- Container purpose and functionality
- Hardware requirements
- Environment variables
- Usage examples
- Configuration options
- Troubleshooting tips

## Adding a New Container

### 1. Choose a Container Slug
- Use lowercase letters, numbers, and hyphens only
- Be descriptive but concise
- Examples: `mqtt-broker`, `yolov8-detector`, `node-red-automation`

### 2. Create the Directory Structure
```bash
mkdir -p containers/your-container-slug
cd containers/your-container-slug
```

### 3. Create Required Files
Start with the `meta.json` file, then create your `Dockerfile`, `entrypoint.sh`, and `README.md`.

### 4. Validate Your Container
Run the validation script to ensure your container meets requirements:
```bash
npm run validate
```

### 5. Test Building
Test that your container builds successfully:
```bash
docker build -t test/your-container-slug containers/your-container-slug/
```

## Testing Your Container

### Local Testing
1. **Build the container**:
   ```bash
   docker build -t streamdeploy/your-container-slug containers/your-container-slug/
   ```

2. **Run the container**:
   ```bash
   docker run --rm streamdeploy/your-container-slug
   ```

3. **Test with environment variables** (if applicable):
   ```bash
   docker run --rm -e VAR_NAME=value streamdeploy/your-container-slug
   ```

### Multi-Architecture Testing
If your container supports multiple architectures, test on different platforms or use Docker buildx:

```bash
docker buildx build --platform linux/amd64,linux/arm64 containers/your-container-slug/
```

### Hardware-Specific Testing
For hardware-specific containers (Raspberry Pi, Jetson, etc.), test on the actual target hardware when possible.

## Submitting a Pull Request

1. **Commit your changes**:
   ```bash
   git add .
   git commit -m "feat: add your-container-slug container"
   ```

2. **Push to your fork**:
   ```bash
   git push origin feature/your-container-name
   ```

3. **Create a Pull Request** on GitHub with:
   - Clear title describing the change
   - Completed PR template
   - Screenshots or examples (if applicable)

## Review Process

### Automatic Reviews
- **Code Owners**: @jl-codes and @clement880101 are automatically assigned as reviewers
- **CI Checks**: Automated tests validate container structure and build process

### Review Criteria
Reviewers will check for:
- ✅ Proper container structure and required files
- ✅ Valid `meta.json` following schema
- ✅ Dockerfile best practices and security
- ✅ Comprehensive documentation
- ✅ Successful builds for specified architectures
- ✅ Appropriate tags and categorization

### Approval Process
- **Required Approvals**: At least one approval from @jl-codes or @clement880101
- **Merge**: Only maintainers can merge approved PRs
- **Feedback**: Address any requested changes promptly

## Container Guidelines

### Best Practices
- **Security**: Use official base images, avoid running as root when possible
- **Size**: Optimize for smaller image sizes
- **Logging**: Use proper logging practices (stdout/stderr)
- **Environment**: Make containers configurable via environment variables
- **Documentation**: Provide clear, comprehensive documentation

### Common Tags
Use relevant tags from this list:
- `ai`, `ml`, `computer-vision`, `inference`
- `iot`, `sensors`, `automation`
- `networking`, `mqtt`, `rtsp`, `streaming`
- `robotics`, `ros`, `navigation`, `slam`
- `edge-computing`, `raspberry-pi`, `jetson`, `coral`
- `logging`, `monitoring`, `data`

### Hardware Categories
- `generic`: Works on standard x86/ARM hardware
- `raspberry-pi`: Optimized for Raspberry Pi
- `jetson`: NVIDIA Jetson devices
- `coral`: Google Coral Edge TPU
- `rk3588`: Rockchip RK3588 devices

## Code of Conduct

- Be respectful and inclusive
- Provide constructive feedback
- Help others learn and improve
- Follow project guidelines and standards

## Getting Help

- **Issues**: Create an issue for questions or problems
- **Discussions**: Use GitHub Discussions for general questions
- **Documentation**: Check existing container examples for reference

## License

By contributing, you agree that your contributions will be licensed under the same license as the project.

---

Thank you for contributing to StreamDeploy Containers! 🚀
