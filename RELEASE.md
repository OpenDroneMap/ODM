# Releasing ODM

This document describes how a versioned ODM release is produced and how to
verify that every artifact actually shipped.

## What a release produces

A release is cut by pushing a `v<version>` git tag (e.g. `v3.6.1`). Three
workflows react to the tag:

| Workflow | File | Artifact | Destination |
| --- | --- | --- | --- |
| Publish Windows Setup | `.github/workflows/publish-windows.yml` | `ODM_Setup_<version>.exe` (signed) | Attached to the GitHub Release |
| Publish Docker and WSL Images | `.github/workflows/publish-docker.yaml` | `opendronemap/odm:<version>` and `:latest` | Docker Hub |
| Publish Docker GPU Images | `.github/workflows/publish-docker-gpu.yaml` | `opendronemap/odm:gpu` | Docker Hub |

The **only** asset attached to the GitHub Release object is the Windows
installer. The Docker images are pushed to Docker Hub, not to the release.

The installer filename comes from the `VERSION` file (`ODM_Setup_{VERSION}.exe`
in `innosetup.iss`), while the Docker image tag comes from the git tag. Both
must agree, so `VERSION` and the tag have to be bumped together.

## Pre-flight checklist

- [ ] `VERSION` is bumped to the new version on `master`.
- [ ] The latest `master` run of **Publish Windows Setup** is green and its
      `Setup` artifact contains an `.exe`. A tag build reuses the same steps, so
      a green master build is the best predictor that the tagged build will
      attach the installer.
- [ ] The latest `master` run of **Publish Docker and WSL Images** is green.
- [ ] The latest `master` run of **Publish Docker GPU Images** is green.

Do not tag until all three master builds are green. Tagging a broken `master`
reproduces the broken build against the tag and yields a release with missing
artifacts.

## Cutting the release

1. Merge the release commit (with the bumped `VERSION`) to `master` and wait for
   all three publish workflows to go green on that commit.
2. Create the GitHub Release and its `v<version>` tag pointing at that commit.
   `svenstaro/upload-release-action` will also create the release if it does not
   already exist, but creating it up front lets you write the changelog.
3. The tag push triggers all three workflows again. The Windows build takes
   ~1h30m; let it finish — do not cancel it, or the installer will not be
   attached.

## Post-release verification

- [ ] `gh release view v<version> --json assets` lists `ODM_Setup_<version>.exe`.
- [ ] `docker manifest inspect opendronemap/odm:<version>` succeeds.
- [ ] `docker manifest inspect opendronemap/odm:gpu` succeeds and `:latest`
      points at the new version.
- [ ] All three tag-triggered workflow runs concluded with `success`.
