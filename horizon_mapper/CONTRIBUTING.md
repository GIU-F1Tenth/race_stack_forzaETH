# Contributing to Horizon Mapper

Thank you for your interest in contributing to the Horizon Mapper project! This document provides guidelines and information for contributors.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [How to Contribute](#how-to-contribute)
- [Development Setup](#development-setup)
- [Coding Standards](#coding-standards)
- [Testing](#testing)
- [Pull Request Process](#pull-request-process)
- [Issue Reporting](#issue-reporting)

## Code of Conduct

This project follows a standard code of conduct:

- Be respectful and inclusive
- Focus on constructive feedback
- Help create a welcoming environment for all contributors
- Report any unacceptable behavior to the maintainers

## How to Contribute

### Types of Contributions

We welcome various types of contributions:

- **Bug Reports**: Help us identify and fix issues
- **Feature Requests**: Suggest new functionality
- **Code Contributions**: Submit bug fixes and new features
- **Documentation**: Improve README, comments, and guides
- **Testing**: Add or improve test coverage

### Before You Start

1. **Check existing issues** to see if your contribution is already being worked on
2. **Open an issue** to discuss major changes before implementing them
3. **Fork the repository** and create a feature branch

## Development Setup

### Prerequisites

- ROS2 Humble
- Python 3.8+
- Git
- A ROS2 workspace set up

### Setting Up Your Development Environment

1. **Fork and clone the repository**:
   ```bash
   git clone https://github.com/your-username/horizon_mapper.git
   cd horizon_mapper
   ```

2. **Create a development branch**:
   ```bash
   git checkout -b feature/your-feature-name
   ```

3. **Set up the ROS2 workspace**:
   ```bash
   cd ~/ros2_ws/src
   ln -s /path/to/your/horizon_mapper .
   cd ~/ros2_ws
   colcon build --packages-select horizon_mapper
   source install/setup.bash
   ```

4. **Install development dependencies**:
   ```bash
   pip install pytest flake8 black isort
   ```

## Coding Standards

### Python Style Guide

We follow PEP 8 with some specific guidelines:

- **Line Length**: Maximum 100 characters
- **Indentation**: 4 spaces (no tabs)
- **Naming Conventions**:
  - Classes: `CamelCase`
  - Functions/Variables: `snake_case`
  - Constants: `UPPER_SNAKE_CASE`

### Code Formatting

Use `black` for automatic formatting:

```bash
black horizon_mapper/
```

Use `isort` for import sorting:

```bash
isort horizon_mapper/
```

### Linting

Run `flake8` for style checking:

```bash
flake8 horizon_mapper/
```

### Documentation

- **Docstrings**: Use Google-style docstrings for all functions and classes
- **Comments**: Add comments for complex logic
- **Type Hints**: Use type hints where appropriate

Example docstring:

```python
def process_trajectory(self, trajectory_data: List[Dict]) -> bool:
    """Process trajectory data and validate format.
    
    Args:
        trajectory_data: List of trajectory points with x, y, v, theta keys
        
    Returns:
        True if processing successful, False otherwise
        
    Raises:
        ValueError: If trajectory data format is invalid
    """
```

## Testing

### Running Tests

Run the full test suite:

```bash
cd ~/ros2_ws
colcon test --packages-select horizon_mapper
colcon test-result --verbose
```

Run specific tests:

```bash
python -m pytest test/test_specific_module.py
```

### Writing Tests

- **Unit Tests**: Test individual functions and classes
- **Integration Tests**: Test component interactions
- **Test Coverage**: Aim for >80% coverage

Example test:

```python
import unittest
from horizon_mapper.horizon_mapper_node import HorizonMapperNode

class TestHorizonMapper(unittest.TestCase):
    def setUp(self):
        self.node = HorizonMapperNode()
    
    def test_trajectory_validation(self):
        """Test trajectory validation functionality."""
        valid_state = self.create_valid_vehicle_state()
        self.assertTrue(self.node._validate_vehicle_state(valid_state))
```

### Test Data

- Keep test data files small and focused
- Use realistic but simplified data
- Document test data sources and formats

## Pull Request Process

### Before Submitting

1. **Update your branch** with the latest main branch:
   ```bash
   git checkout main
   git pull origin main
   git checkout your-feature-branch
   git rebase main
   ```

2. **Run tests** and ensure they pass:
   ```bash
   colcon test --packages-select horizon_mapper
   ```

3. **Check code style**:
   ```bash
   black --check horizon_mapper/
   flake8 horizon_mapper/
   ```

### PR Requirements

- **Descriptive Title**: Clearly describe what the PR does
- **Detailed Description**: Explain the motivation and approach
- **Issue Reference**: Link to related issues (if applicable)
- **Testing**: Describe how the changes were tested
- **Breaking Changes**: Highlight any breaking changes

### PR Template

```markdown
## Description
Brief description of changes

## Motivation and Context
Why is this change required? What problem does it solve?

## How Has This Been Tested?
Describe testing performed

## Types of Changes
- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to change)
- [ ] Documentation update

## Checklist
- [ ] My code follows the code style of this project
- [ ] I have added tests to cover my changes
- [ ] All new and existing tests passed
- [ ] I have updated the documentation accordingly
```

### Review Process

1. **Automatic Checks**: CI/CD pipeline runs tests and style checks
2. **Maintainer Review**: Code review by project maintainers
3. **Feedback Address**: Implement requested changes
4. **Approval and Merge**: PR is merged once approved

## Issue Reporting

### Bug Reports

Use the bug report template:

```markdown
**Describe the bug**
A clear description of what the bug is.

**To Reproduce**
Steps to reproduce the behavior

**Expected behavior**
What you expected to happen

**Environment:**
- ROS2 Version: [e.g., Humble]
- OS: [e.g., Ubuntu 22.04]
- Python Version: [e.g., 3.8]

**Additional context**
Any other context about the problem
```

### Feature Requests

Use the feature request template:

```markdown
**Is your feature request related to a problem?**
A clear description of what the problem is

**Describe the solution you'd like**
A clear description of what you want to happen

**Describe alternatives you've considered**
Any alternative solutions or features you've considered

**Additional context**
Any other context about the feature request
```

## Release Process

### Version Numbering

We use Semantic Versioning (SemVer):
- `MAJOR.MINOR.PATCH`
- Major: Breaking changes
- Minor: New features (backward compatible)
- Patch: Bug fixes (backward compatible)

### Release Checklist

- [ ] Update version in `setup.py` and `package.xml`
- [ ] Update CHANGELOG.md
- [ ] Tag release: `git tag v1.0.0`
- [ ] Push tags: `git push origin --tags`

## Getting Help

- **GitHub Issues**: For bug reports and feature requests
- **GitHub Discussions**: For questions and general discussion
- **Email**: Direct contact at mo7ammed3zab@outlook.com

## Recognition

Contributors will be acknowledged in:
- CONTRIBUTORS.md file
- Release notes
- Project documentation

Thank you for contributing to Horizon Mapper! 🚗🏁
