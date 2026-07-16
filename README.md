# F1-Tenth-URI
Welcome to F1-Tenth!
This is the GitHub this we use for this club. We will be useing this to manage tasks and to share information.
Below is a guide on how to use Github.

## 1. One-Time Setup
 
Install Git if you don't have it: https://git-scm.com/downloads
 
Clone the repository to your computer:
 
```bash
git clone <repository-url>
cd <repository-name>
```
 
Set your name and email (only needed once per computer):
 
```bash
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```
 
## 2. Get the Latest Code
 
Before starting any work, make sure your local copy of `main` is up to date:
 
```bash
git checkout main
git pull
```
 
## 3. Create a Branch
 
Never make changes directly on `main`. Instead, create a new branch for your work. Name it something short and descriptive.
 
```bash
git checkout -b your-name/short-description
```
 
Example:
 
```bash
git checkout -b jim/fix-login-bug
```
 
This creates the branch and switches you onto it.
 
## 4. Make Your Changes
 
Edit files as needed in your code editor. When you're ready to save your progress, check what's changed:
 
```bash
git status
```
 
Stage the files you want to include:
 
```bash
git add <file-name>
```
 
Or stage everything that changed:
 
```bash
git add .
```
 
Commit the staged changes with a short message describing what you did:
 
```bash
git commit -m "Fix login bug on the sign-in page"
```
 
You can repeat the add/commit steps as many times as you like while you work.
 
## 5. Push Your Branch to GitHub
 
The first time you push a new branch:
 
```bash
git push -u origin your-name/short-description
```
 
After that, subsequent pushes on the same branch just require:
 
```bash
git push
```
 
## 6. Open a Pull Request
 
A pull request (PR) is how you propose merging your branch into `main`.
 
1. Go to the repository on GitHub.com.
2. You should see a banner suggesting "Compare & pull request" for your recently pushed branch — click it. (If not, go to the "Pull requests" tab and click "New pull request.")
3. Confirm the base branch is `main` and the compare branch is yours.
4. Add a title and description explaining what the change does and why.
5. Click "Create pull request."
## 7. Review and Merge
 
- Ask a teammate to review the PR, or review it yourself if you're working solo.
- GitHub will show whether the branch can merge cleanly. If it shows conflicts, see the section below.
- Once approved, click "Merge pull request," then "Confirm merge."
- Click "Delete branch" afterward to keep things tidy (this only deletes the branch, not your commits — they live on in `main`).
## 8. Sync Up Afterward
 
Back on your computer, switch to `main` and pull the merged changes:
 
```bash
git checkout main
git pull
```
 
You can now delete your local branch if you're done with it:
 
```bash
git branch -d your-name/short-description
```
 
## Handling Merge Conflicts
 
A conflict happens when your branch and `main` both changed the same lines of a file. Git will tell you which files are affected.
 
1. Pull the latest `main` into your branch:
```bash
   git checkout your-name/short-description
   git pull origin main
```
 
2. Open the conflicting files. Git marks conflicts like this:
```
   <<<<<<< HEAD
   your version of the code
   =======
   the incoming version of the code
   >>>>>>> main
```
 
3. Edit the file to keep the correct content and remove the `<<<<<<<`, `=======`, and `>>>>>>>` markers.
4. Stage and commit the resolved files:
```bash
   git add .
   git commit -m "Resolve merge conflict"
```
 
5. Push again:
```bash
   git push
```
 
## Quick Reference
 
| Task | Command |
|---|---|
| Get latest `main` | `git checkout main && git pull` |
| Create a branch | `git checkout -b your-name/short-description` |
| Check changed files | `git status` |
| Stage changes | `git add .` |
| Commit changes | `git commit -m "message"` |
| Push a new branch | `git push -u origin your-name/short-description` |
| Push again later | `git push` |
| Switch branches | `git checkout branch-name` |
 
## Golden Rules
 
- Never commit directly to `main`. Always work on a branch.
- Pull `main` before starting new work so you're building on the latest code.
- Write clear commit messages — future you (and everyone else) will thank you.
- Open a pull request for every change, even small ones, so others can review it.
- If you're unsure about anything, ask before merging — it's much easier to fix a problem before it's on `main`.
