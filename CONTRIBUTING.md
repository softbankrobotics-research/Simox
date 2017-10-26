# Contributing to Simox as a GitLab Team Member

## Developer

### Creating a Merge Request

1. clone Simox as indicated in [README.md](README.md)
2. create a branch in the specific SimoxX package with `git checkout -b <branchname>`
3. develop your feature and commit the changes to the branch (create multiple smaller commits if possible)
4. push the changes to a remote branch on GitLab with `git push -u origin <branchname>`
5. click the `Create Merge Request` button which appears on the top of the page when you visit https://gitlab.com/Simox/simox
6. check that your newly created branch is set as the source branch of the merge request
7. check that `master` is set as the target branch of the merge request
8. select your advisor as assignee (leave the field empty if unsure)
9. click the `SUBMIT NEW MERGE REQUEST` button

### After Creating a Merge Request

After submitting the merge request it might take a little bit for someone to review the code and accept the request.
Any comments, changes, or suggestions made on the request should be addressed.

It is no problem at all to push additional changes to the branch in the repository.
These changes are added to the merge request automatically.

#### Resolving Conflicts between master branch and your Merge Request

During the lifetime of a merge request, automatic merging via
GitLab web interface might be disabled due to merge conflicts.
Resolving these conflicts requires your branch to get in sync with the latest
`master` branch which can be achieved in two ways.

First update your `master` branch using `git checkout master & git pull`.
Then proceed with one of the following steps:

* unexpirienced Git users:
    1. `git checkout <your-merge-request-branch>`
    2. `git merge master`
    3. resolve conflicts and commit the changes
    4. `git push`

* experienced Git users (you should know what `git rebase` is and what effects it has):
    1. `git checkout <your-merge-request-branch>`
    2. `git rebase master` (requires resolving conflicts during rebase)
    3. `git push --force` (only do a `push --force` on merge request branches, where this is expected to happen)

## Repository Master/Owner

### Reviewing Merge Requests

1. go to the repository of the specific Simox package
2. open the merge request and check the description and the changes (diff view)
3. leave general comments on the `Discussion` tab if something is missing, wrong, or needs to be changed
4. leave more detailed comments on specific lines of the sourcecode in the `Changes` tab
5. once everything is ready to be merged click the `Remove source branch` checkbox and afterwards the `ACCEPT MERGE REQUEST` button

### Releasing a new Simox version

1. check out or update the `master' branch
2. icrease the version number X.Y.Z in the file config.cmake 
3. commit the changed config.cmake file
4. add a tag with 'git tag -a "vX.Y.Z" -m"vX.Y.Z"'
5. push with 'git push --tags'
6. The H2T jenkins server will automatically build a new simox package