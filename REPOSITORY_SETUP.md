# Repository Configuration Guide

This guide outlines the repository settings you need to configure to enable proper contribution controls for the StreamDeploy Containers project.

## Branch Protection Rules

### 1. Navigate to Branch Protection Settings
Go to: `https://github.com/StreamDeploy/streamdeploy-containers/settings/branches`

### 2. Add Rule for `main` Branch
Click "Add rule" and configure:

#### Branch name pattern
```
main
```

#### Protection Settings
✅ **Require a pull request before merging**
- ✅ Require approvals: `1`
- ✅ Dismiss stale PR approvals when new commits are pushed
- ✅ Require review from code owners
- ✅ Restrict pushes that create files that have a path matching a CODEOWNERS pattern

✅ **Require status checks to pass before merging**
- ✅ Require branches to be up to date before merging
- Required status checks (add these):
  - `Validate Container Structure`
  - `Validate Container Metadata`
  - `Test Container Builds`
  - `Lint Dockerfiles`
  - `Check Documentation`
  - `Validation Summary`

✅ **Require conversation resolution before merging**

✅ **Restrict pushes to matching branches**
- Add exceptions for:
  - `jl-codes` (you)
  - `clement880101` (Clement)

✅ **Allow force pushes** (unchecked - disabled)

✅ **Allow deletions** (unchecked - disabled)

## Repository Settings

### 1. General Settings
Go to: `https://github.com/StreamDeploy/streamdeploy-containers/settings`

#### Pull Requests
- ✅ Allow merge commits
- ✅ Allow squash merging (recommended as default)
- ✅ Allow rebase merging
- ✅ Always suggest updating pull request branches
- ✅ Allow auto-merge
- ✅ Automatically delete head branches

#### Issues
- ✅ Issues (should be enabled)

#### Discussions
- ✅ Discussions (optional, but recommended for community questions)

### 2. Collaborators and Teams
Go to: `https://github.com/StreamDeploy/streamdeploy-containers/settings/access`

#### Repository Access
- **Base permissions**: `Read` (allows anyone to fork and create PRs)
- **Maintainers**: 
  - `jl-codes` (Admin)
  - `clement880101` (Admin or Maintain)

### 3. Actions Settings
Go to: `https://github.com/StreamDeploy/streamdeploy-containers/settings/actions`

#### Actions permissions
- ✅ Allow all actions and reusable workflows

#### Workflow permissions
- ✅ Read and write permissions
- ✅ Allow GitHub Actions to create and approve pull requests

## Labels Configuration

### 1. Navigate to Labels
Go to: `https://github.com/StreamDeploy/streamdeploy-containers/labels`

### 2. Create/Update Labels
Ensure these labels exist (create if missing):

#### Priority Labels
- `priority: low` - #d4edda (light green)
- `priority: medium` - #fff3cd (light yellow)  
- `priority: high` - #f8d7da (light red)
- `priority: critical` - #721c24 (dark red)

#### Type Labels
- `bug` - #d73a49 (red)
- `enhancement` - #a2eeef (light blue)
- `new-container` - #0075ca (blue)
- `documentation` - #0052cc (dark blue)
- `good first issue` - #7057ff (purple)
- `help wanted` - #008672 (teal)

#### Status Labels
- `needs review` - #fbca04 (yellow)
- `needs testing` - #d4c5f9 (light purple)
- `ready to merge` - #0e8a16 (green)
- `blocked` - #b60205 (dark red)

## Security Settings

### 1. Code Security and Analysis
Go to: `https://github.com/StreamDeploy/streamdeploy-containers/settings/security_analysis`

#### Recommended Settings
- ✅ Dependency graph
- ✅ Dependabot alerts
- ✅ Dependabot security updates
- ✅ Code scanning (if available)
- ✅ Secret scanning

## Notifications

### 1. Notification Settings
Ensure you and Clement receive notifications for:
- ✅ Pull requests
- ✅ Issues
- ✅ Releases
- ✅ Actions workflow runs

## Testing the Setup

### 1. Create a Test PR
1. Create a test branch with a small change
2. Open a PR to `main`
3. Verify:
   - You and Clement are automatically assigned as reviewers
   - PR validation workflow runs
   - Branch protection prevents merging without approval
   - All status checks must pass

### 2. Test Issue Templates
1. Go to Issues → New Issue
2. Verify all three templates appear:
   - New Container Request
   - Bug Report  
   - Feature Request

### 3. Test Contributor Flow
1. Have someone fork the repository
2. Let them create a PR with a new container
3. Verify the full validation and review process works

## Maintenance

### Regular Tasks
- Review and update labels as needed
- Monitor workflow performance and adjust as needed
- Update branch protection rules if new checks are added
- Review contributor feedback and improve templates

### Quarterly Reviews
- Review access permissions
- Update documentation based on contributor feedback
- Assess if additional automation is needed

## Troubleshooting

### Common Issues
1. **Status checks not appearing**: Ensure the workflow names match exactly in branch protection
2. **CODEOWNERS not working**: Verify file is in `.github/CODEOWNERS` and usernames are correct
3. **PR template not showing**: Ensure file is in `.github/pull_request_template.md`

### Getting Help
- Check GitHub's documentation on branch protection
- Review Actions workflow logs for validation issues
- Test changes in a fork first before applying to main repository

---

Once you've completed these settings, the repository will be fully configured for community contributions with proper approval controls.
