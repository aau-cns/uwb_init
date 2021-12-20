## (Release Version) - (Release Name)
<!-- Updates to release should at least increase the MINOR version number.-->

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
  - [ ] Assign the a designated Maintainer (Merge Request Handler) and add the ~"todo" label (this must be a different person).
  - [ ] Checked the `CHANGELOG.md` to account for all release changes.

**Do not** add the ~"feature", ~"bug", or ~"enhancement" labels if you are only updating documentation.

---

**This MR follows the AAU CNS MR Guidelines available in the [Intranet](https://intranet.aau.at/display/aauintsycns/GitLab+Manual) and corresponding [GitLab Repository](https://gitlab.aau.at/aau-cns/standard/gitlab_setup).**

---

### Review checklist (for project maintainer)
<!-- THIS SECTION IS FOR THE PROJECT MAINTAINER ONLY!!!! -->

- [ ] Merge Request Review
  - [ ] Check if the branches for merging is correct.
  - [ ] Ensure the patch version is correct.
  - [ ] Checked if all information is given and MR tasks performed.
  - [ ] Unset the ~"request" label.

- [ ] Code Review and Test
  - [ ] Review by a code reviewer or other selected colleague to confirm accuracy, clarity, and completeness. This can be skipped for minor fixes without substantive content changes.
  - [ ] Check if all necessary tests have been performed
      - If not assign and complete a test issue (to yourself or others)
      - Check also if tests on different hardware (ARM) were successful
  - [ ] Check if the code still compiles
  - [ ] Set the ~"awaiting-merge" label upon test completion.

- [ ] Merging
  - [ ] If there has not been a documentation update, make the documentation is updated accordingly.
  - [ ] Remove `WIP:` in title upon merge request review completion.
  - [ ] Unset the ~"awaiting-merge" label.
  - [ ] Performed `--no-ff` merge (_check all tasks before pushing!!!_)
      - Copy the above `Closes #XYZ` line into the merge commit.


/label ~"request" ~"document"
/spend 10m
/estimate 1h
