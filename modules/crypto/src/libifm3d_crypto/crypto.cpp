/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/crypto/crypto.h>
#include <ifm3d/common/err.h>
#include <sodium/crypto_box.h>
#include <sodium/randombytes.h>
#include <stdexcept>

ifm3d::SealedBox::SealedBox(
  const std::vector<uint8_t> public_key,
  std::optional<const std::vector<uint8_t>> private_key)
  : public_key_(public_key),
    private_key_(private_key)
{}

ifm3d::SealedBox::~SealedBox() {}

std::vector<uint8_t>
ifm3d::SealedBox::Encrypt(const std::string& plaintext)
{
  std::vector<uint8_t> ciphertext(plaintext.length() + crypto_box_SEALBYTES);

  int ret = crypto_box_seal(
    reinterpret_cast<unsigned char*>(ciphertext.data()),
    reinterpret_cast<const unsigned char*>(plaintext.data()),
    plaintext.length(),
    reinterpret_cast<const unsigned char*>(this->public_key_.data()));

  if (ret != 0)
    {
      throw ifm3d::Error(IFM3D_CRYPTO_ERROR,
                         "Encryption failed, unable to encrypt message.");
    }

  return ciphertext;
}

std::string
ifm3d::SealedBox::Decrypt(const std::vector<std::uint8_t>& ciphertext)
{
  if (this->private_key_ == std::nullopt)
    {
      throw ifm3d::Error(IFM3D_CRYPTO_ERROR,
                         "Private key is not set, unable to decrypt message.");
    }

  std::string plaintext(ciphertext.size() - crypto_box_SEALBYTES, '\0');

  int ret = crypto_box_seal_open(
    reinterpret_cast<unsigned char*>(plaintext.data()),
    reinterpret_cast<const unsigned char*>(ciphertext.data()),
    ciphertext.size(),
    reinterpret_cast<const unsigned char*>(this->public_key_.data()),
    reinterpret_cast<const unsigned char*>(this->private_key_.value().data()));

  if (ret != 0)
    {
      throw ifm3d::Error(IFM3D_CRYPTO_ERROR,
                         "Decryption failed, unable to decrypt message.");
    }

  return plaintext;
}

std::vector<uint8_t>
ifm3d::RandomNonce()
{
  std::vector<uint8_t> nonce(crypto_box_NONCEBYTES);
  randombytes_buf(nonce.data(), crypto_box_NONCEBYTES);
  return nonce;
}