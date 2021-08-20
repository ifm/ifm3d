# Generating and uploading ssh keys

It is possible to connect via `ssh` to the VPU, using the user `oem`. The `oem` user has however no default password. To still access the VPU a ssh key is needed. Fur further information look into the official documentation [here](*insert link*). The python script `uploadSSHKey.py` is a sample python script generating ssh keys and using `ifm3dpy` for the deployment on the VPU.

`uploadSSHKey.py` also consists of an CLI. Use `python3 uploadSSHKey.py --help` for further information.

## Using the CLI

The easiest way to generate, save and upload the keys is using the CLI.

On posix systems, you can add the executable flag to the python script and
run it directly:

```bash
devoegse@Ubuntu:~/Git/ifm3d/examples/o3r/oem/ssh$ chmod +x uploadSSHKey.py
devoegse@Ubuntu:~/Git/ifm3d/examples/o3r/oem/ssh$ ./uploadSSHKey.py --help
```

```console
python3 -m uploadSSHKeys.py [key name] [passphrase] --savekeys --uploadkey
```

```console
python3 -m uploadSSHKeys.py id_o3r oem --savekeys --uploadkey
```

Use `--saveKeys` as option to save the generated keys. Default: True

Use `--uploadkeys` as option to upload the public key to the O3R. Default: False

## Generating ssh keys

Use `generateKeys` to generate the ssh keys:

```python
"""
Generate ssh keys with key size 4096 bytes.

:key_name:str:  File name for the key (typically used: id_rsa)
:passphrase:str:    Passphrase for the ssh connection, saved within the keys

:return:dict:   Return the keys within a dictionary 'private'/'public'
"""
```

## Save the keys

Use `saveKeys` to save the keys within the ssh directory:

```python
"""
Save the keys on the system.

:keys:dict: Dictionary containing the keys
:key_name:  File name for the keys
"""
```

## Uploading keys

Use `upload` to upload the public key:

```python
"""
Add the new key to the config and upload the configuration back to the
O3R.

:key:bytes: Public key to forward to the O3R.
"""
```