## Docker dev containers
Multiple development containers are available. You can pull them using the following command, with the relevant tag:
```
$ docker pull ghcr.io/ifm/ifm3d:TAG
```
The following tags are available:
- `stable`: this is the {{ '[latest released version]({})'.format(ifm3d_latest_tag_url) }}. 
- `latest`: this is a nightly build with the latest version of ifm3d available on the {{ ifm3d_main_branch }} branch. This typically contains work in progress.
- VERSION: you can specify a specific version of ifm3d, for instance, use:
```
$ docker pull ghcr.io/ifm/ifm3d:v0.93.0
```
The full list of available container versions is available {{ '[here]({})'.format(ifm3d_containers_list_url) }}.

For each tag, several suffixes are available:
- `ubuntu`: an Ubuntu-based build, available both for amd64 and aarch64.
- `l4t`: a build based on nvidia's Linux 4 Tegra, only available for aarch64 architecture.

Each of these suffixes can be combined with a tag to get a specific version for a specific OS. For instance, you can use the tag `v0.93.0-l4t` to get the Linux 4 Tegra based build of ifm3d version 0.93.0.
When no suffix is added to the tag, the default build is used, which is Ubuntu based.

To explicitly specify the architecture required for the image, one can use:
```
docker pull --platform=linux/arm64 ghcr.io/ifm/ifm3d:stable
```
Suffixes are also available to define the platform. For instance, one can use the tag `stable-ubuntu-arm64`.

For more detailed documentation on using docker containers with the O3R platform, you can refer to [this section](documentation/docker/README:Docker%20on%20O3R).