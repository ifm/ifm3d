// -*- c++ -*-
/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2
 */

#ifndef IFM3D_CRYPTO_CRYPTO_H
#define IFM3D_CRYPTO_CRYPTO_H

#include <string>
#include <vector>
#include <optional>
#include <cstdint>

namespace ifm3d
{
  class SealedBox
  {
  public:
    SealedBox(
      const std::vector<uint8_t> public_key,
      std::optional<const std::vector<uint8_t>> private_key = std::nullopt);

    ~SealedBox();

    std::vector<uint8_t> Encrypt(const std::string& plaintext);
    std::string Decrypt(const std::vector<uint8_t>& ciphertext);

  private:
    std::vector<uint8_t> public_key_;
    std::optional<std::vector<uint8_t>> private_key_;
  };

  std::vector<uint8_t> RandomNonce();

} // end: namespace ifm3d

#endif // IFM3D_CRYPTO_CRYPTO_H
