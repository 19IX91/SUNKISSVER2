# Upload SUNKISSVER2 to GitHub

The `SUNKISSVER2_GITHUB` folder is the repository root. On Linux, use the `.tar.gz` package to preserve the controller executable bit. The `.zip` is an alternative; after extracting it, run `chmod +x bin/r1_ctrl_autostart_linux_x86_64`. **Extract either package first**; GitHub does not unpack an archive added as a repository file.

## Upload commands

Git and GitHub CLI (`gh`) are downloaded under `/home/ix/Projects/github_upload_tools`. In each new terminal, load them first. Then run:

```bash
source /home/ix/Projects/github_upload_tools/env.sh
cd /home/ix/Projects/SUNKISSVER2_GITHUB
git init -b main
git add .
git commit -m "Archive SUNKISSVER2 controller and validation"
gh auth login
gh repo create SUNKISSVER2 --private --source=. --remote=origin --push
```

`gh auth login` opens GitHub authentication. If `git commit` asks for an identity, set `git config --global user.name "Your Name"` and `git config --global user.email "your-github-email@example.com"`, then repeat the commit. The final command creates a **private** repository and pushes the commit. If the repository name is taken, choose another name in the final command.

## Verify the upload

After the push, run:

```bash
gh repo view SUNKISSVER2 --web
```

Check that `README.md`, `deploy/robots/r1/main.cpp`, `deploy/robots/r1/config/sunkiss.yaml`, `deploy/robots/r1/config/policy/tracking/sunkiss/v1/exported/policy.onnx`, `mujoco/`, and `VERSION_NOTES.md` appear in the repository.

The archive has no file larger than GitHub's normal 100 MiB Git limit. Keep the repository private unless you have reviewed the robot control code, the trained policy, logs, and third-party licensing for publication. The latest hardware run ended with the robot down; the README records this result.
