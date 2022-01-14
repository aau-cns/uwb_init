## (Bugfix Branch) - (Bugfix Name)
<!-- This MR updates the develop branch -->

### What does this MR do?
<!-- Briefly describe what this MR is about. -->

### Related issues
<!-- Link related issues below. Insert the issue link or reference after the word "Closes" if merging this should automatically close it. -->
Closes

**When performing this merge, please copy the above `Closes #XYZ` line into the merge commit.**

---

### Author's checklist (required)

- [ ] Ensure that the `WIP:` (work in progress) is present in MR title.

- If you have *Developer* permissions or higher:
  - [ ] Ensure that the releated issues are added.
  - [ ] Apply the corresponding DevOps labels (e.g., ~"feature", ~"enhancement", etc.)
  - [ ] Assign the a designated Maintainer (Merge Request Handler) and add the ~"todo" label (this should be a different person).
  - [ ] Checked the `CHANGELOG.md` to account for all release changes.

**Do not** add the ~"feature", ~"bug", or ~"enhancement" labels if you are only updating documentation.

---

**This MR follows the AAU CNS MR Guidelines available in the [Intranet](https://intranet.aau.at/display/aauintsycns/GitLab+Manual) and corresponding [GitLab Repository](https://gitlab.aau.at/aau-cns/standard/gitlab_setup).**

---

### Review checklist (for project maintainer/developer)
<!-- THIS SECTION IS FOR THE PROJECT MAINTAINER ONLY!!!! -->

- [ ] Merge Request Review
  - [ ] Check if the branches for merging is correct.
  - [ ] Checked if all information is given and MR tasks performed.
  - [ ] Unset the ~"request" label.
- [ ] Code Review and Test
  - [ ] Short review by a maintainer before merging to `develop`
  - [ ] Check if all tests have been performed
      - If not create test issue for the then merged `develop` branch (to yourself or others)
      - Check also if tests on different hardware (ARM) were successful
  - [ ] Check if the code still compiles
  - [ ] Check if the coding style is kept
  - [ ] Set the ~"awaiting-merge" label upon test completion.
- [ ] Merging
  - [ ] If there has not been a documentation update, make sure a documentation issue is created to update accordingly.
  - [ ] Remove `WIP:` in title upon merge request review completion.
  - [ ] Unset the ~"awaiting-merge" label.
  - [ ] Performed `--no-ff` merge (_check all tasks before pushing!!!_)
      - Copy the above `Closes #XYZ` line into the merge commit.


/label ~"request" ~"document" ~"bug"
/spend 10m
/estimate 1h
