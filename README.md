# CS 123 Lab Workspace

All my CS 123 labs, one folder per lab.

- `pd_control_lab/` — week 1
- `forward_kinematics_lab/` — week 2

## Adding a new lab

Clone it, delete its `.git` so it becomes a plain folder, then commit:

```bash
git clone https://github.com/cs123-stanford/<lab_name>
rm -rf <lab_name>/.git
git add <lab_name>
git commit -m "Add <lab_name>"
git push
```
