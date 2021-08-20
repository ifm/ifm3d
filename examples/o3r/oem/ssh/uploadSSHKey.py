#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2020 ifm electronic gmbh
#
# THE PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND.
#

import typer
import getpass
import os
import sys
import ifm3dpy

from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.backends import default_backend as crypto_default_backend

class SSHKeyGen():
    """
    This class uses `cryptography` to generate ssh-key pairs. It can also save
    these keys.
    """
    def __init__(self):
        """
        Save the ssh directory (should work for windows and linux).
        """
        self.ssh_dir = f"{os.path.expanduser('~')}/.ssh/"

    def generateKeys(self, key_name, passphrase):
        """
        Generate ssh keys with key size 4096 bits.

        :key_name:str:  File name for the key (typically used: id_rsa)
        :passphrase:str:    Passphrase for the ssh connection, saved within the keys

        :return:dict:   Return the keys within a dictionary 'private'/'public'
        """
        if key_name in os.listdir(self.ssh_dir):
            raise Exception('Keys already existent')

        else:
            private_key = rsa.generate_private_key(
                backend=crypto_default_backend(),
                public_exponent=65537,
                key_size=4096
            )

            encrypted_private_key = private_key.private_bytes(
                encoding=serialization.Encoding.PEM,
                format=serialization.PrivateFormat.PKCS8,
                encryption_algorithm=serialization.BestAvailableEncryption(bytes(passphrase, 'utf-8')),
            )

            public_key = private_key.public_key().public_bytes(
                encoding=serialization.Encoding.PEM,
                format=serialization.PublicFormat.SubjectPublicKeyInfo,
            )

            keys = {
                'private':encrypted_private_key,
                'public':public_key,
                }

            return keys

    def saveKeys(self, keys, key_name):
        """
        Save the keys on the system.

        :keys:dict: Dictionary containing the keys
        :key_name:  File name for the keys
        """
        for key in keys:
            if key == "private":
                with open(os.open(self.ssh_dir+key_name, os.O_CREAT | os.O_WRONLY, 0o600), "w") as key_file:
                    key_file.write(keys[key].decode())
            else:
                with open(os.open(self.ssh_dir+key_name+".pub", os.O_CREAT | os.O_WRONLY, 0o600), "w") as key_file:
                    key_file.write(keys[key].decode())

class UploadKey():
    """
    This class is using the ifm3dpy (Python bindings ifm3d) for receiving
    the VPU (O3R) configuration. Add a new key, and upload the changed
    configuration.
    """
    def __init__(self):
        self.o3r = ifm3dpy.O3RCamera()

    def upload(self, key):
        """
        Add the new key to the config and upload the configuration back to the
        O3R.

        :key:bytes: Public key to forward to the O3R.
        """
        config = self.o3r.to_json()
        old_keys = config['device']['network']['authorized_keys']
        new_keys = old_keys + ',' + str(key, encoding='UTF-8')
        config['device']['network']['authorized_keys'] = new_keys
        self.o3r.from_json(config)

app = typer.Typer()

@app.command()
def generate(
    key_name:str=typer.Argument('id_rsa',help="Name of the key(s)"),
    passphrase:str=typer.Argument("oem",help="A passphrase is needed for the private key connection via ssh"),
    saveKeys:bool=typer.Option(True,help="Save the keys into the .ssh folder"),
    uploadKey:bool=typer.Option(False,help="Upload the public key to the VPU via ifm3d"),
    ):
    """
    Generate, save and upload ssh-key pairs.

    Use --saveKeys as option to save the generated keys.

    Use --uploadkeys as option to upload the public key to the O3R.
    """

    ssh = SSHKeyGen()

    try:
        typer.echo("Generate keys")
        keys = ssh.generateKeys(key_name, passphrase)

        if saveKeys:
            typer.echo("Save keys")
            ssh.saveKeys(keys, key_name)

        if uploadKey:
            typer.echo("Upload keys")
            upload = UploadKey()
            upload.upload(keys['public'])

    except Exception as e:
        typer.echo(e)

if __name__ == "__main__":
    app()