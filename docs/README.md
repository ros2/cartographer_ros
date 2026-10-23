# Documentation on GitHub Pages

The files in `source/` are reStructuredText input for Sphinx. GitHub's file
preview does not expand Sphinx directives such as `toctree`; the rendered site
provides topic links, navigation, and search.

The `.github/workflows/documentation.yml` workflow builds and publishes the site
on GitHub. No local documentation build or ROS installation is required.

## One-time repository setup

1. Commit and push the workflow, `docs/requirements.txt`, and documentation changes
   to the `lyrical` branch.
2. In the repository, open **Settings → Pages → Build and deployment** and set
   **Source** to **GitHub Actions**.
3. In **Settings → Environments → github-pages**, ensure deployment protection
   rules allow the `lyrical` branch. GitHub may create this environment on the
   first workflow run.
4. Open **Actions → Documentation → Run workflow**, choose `lyrical`, and run it.
   If the initial push ran before Pages was enabled, rerun the failed workflow.

After the first successful deployment, the documentation is available at:

**https://slgrobotics.github.io/cartographer_ros/**

## Subsequent updates

Edit the `.rst` files in `docs/source/` and push them to `lyrical`. GitHub rebuilds
and publishes the site automatically. Pull requests targeting `lyrical` build the
HTML for validation but never deploy it. Manual runs deploy only from `lyrical`.

Read warnings and failures in the Documentation workflow's build logs. Sphinx is
pinned in `requirements.txt`; update that pin deliberately when upgrading.
Generated HTML is uploaded as a Pages artifact and is not committed to Git.
`docs/COLCON_IGNORE` does not affect this workflow.

Some inherited pages still describe upstream ROS 1 workflows; publishing the site
does not update those instructions. See the repository's `README_Lyrical.md` for
fork-specific Lyrical notes.
