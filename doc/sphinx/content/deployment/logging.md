
# Logging in ifm3d

The ifm3d API and its CLI logs to STD OUT in versions < 1.3 with high verbosity. To change this behavior one can apply these changes:
```sh
# Write the IFM3D logs to the stderr instead to a file
export GLOG_logtostderr=1

# Do not log anything below fatal
export GLOG_minloglevel=3
```
