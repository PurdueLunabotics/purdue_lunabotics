# Contributing!

Thanks for taking the time to contribute to the repo! Here are some rules that define our repo structure. Following these rules will save you and others a lot of pain!

### Branch Naming Conventions

#### Permanent Branches
- `master`
  - Represents current working state of the repo
- `develop`
  - Used for integration of features and individual contrbutions

#### Temporary Branches

- `feature/feature-name`
  - Use this for general (long-term) features that **multiple people** will touch (i.e `feature/graphslam`)
- `user/your-gh-username/<contribution-name>`
  - Use this for small bug fixes and **individual** contributions (i.e `user/johncena/quick-encoder-fix`)

For all temporary branches:
- Open PR to `develop` or respective `feature` branch, **NOT** `master`
### General Individual Development Workflow

> Designed to try to avoid the [Bike Shed Effect](https://personalexcellence.co/blog/bike-shed-effect/)

1. Create new `user/your-name` branch from:
   - `feature` branch that you're working on (optional, create new `feature` branch if adding to new feature)
   - `develop` if quick fix or individual contribution
2. Develop base algo logic (no formatting, comments, and fluff, is necessary)
3. Open PR on Github merging to `feature` branch or `develop`
   - Add screenshots/results on the PR for proof that your algo works as expected (test results, accuracy, pictures of plots)
4. Request algo review on Github from admins
5. Make changes to improve code quality/readability
6. Request final review on GH
7. [Squash all commits into one with commit with a useful commit message](https://gist.github.com/raghavauppuluri13/cc84469719528136fd17d93dc1a2c745)
8. Merge on GH

### Integration testing Workflow

After integration testing, a PR is made from `develop` to `master` with unit testing and integration testing results in PR. Once approved, the `master` branch will be updated to the latest `develop`

# Coding Standards

We have pre-commit hooks to help you adhere to basic coding standards.

## Quickstart
1. Install pre-commit `pip install pre-commit`
2. In the repo root directory, install the pre-commit hook `pre-commit install` 
3. Make changes
4. On `git commit`, precommit will tell you if you violated some coding standards, if so fix, stage, and recommit

> NOTE: if you are committing quickly and want to skip the pre-commit check do `git commit --no-verify`. Avoid doing this often, otherwise you'll go through a lot of pain later!


#### Python Source
Try to adhere to [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html)

TLDR:
- use type hinting for every function and class variable
- to simplify all the fluff style standards, use [black](https://www.freecodecamp.org/news/auto-format-your-python-code-with-black/) autoformatter

#### C++ Source
Similar to python except [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)

TLDR:
- to simplify all the fluff style standards, use [clang-format](https://leimao.github.io/blog/Clang-Format-Quick-Tutorial/) autoformatter

## Code Review Standards

To ensure feedback from code reviews are actionable and clear, 
we follow these standards: https://conventionalcomments.org/

## ROS Naming Conventions

### Package Layout
```
- lunabot_PACKAGE
  - include
    - lunabot_PACKAGE
      - *.h header files
  - scripts
    - Python ROS Node executables (use *.py extensions please!)
  - src
    - lunabot_PACKAGE (python package for larger library-like code)
      - python package with source .py files
        - can be imported by executables in "scripts"
    - *.cpp source files
  - setup.py 
  - package.xml
  - CMakelists.txt
```

### Everything Else 

- General
  - Follow ROS Naming Conventions: http://wiki.ros.org/ROS/Patterns/Conventions
