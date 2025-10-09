// -*- c++ -*-
/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2
 */

#ifndef IFM3D_CRYPTO_CRYPTO_H
#define IFM3D_CRYPTO_CRYPTO_H

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ifm3d
{
  class SealedBox
  {
  public:
    SealedBox() = default;
    SealedBox(const SealedBox&) = default;
    SealedBox(SealedBox&&) = delete;
    SealedBox& operator=(const SealedBox&) = default;
    SealedBox& operator=(SealedBox&&) = delete;
    SealedBox(const std::vector<uint8_t>& public_key,
              const std::optional<const std::vector<uint8_t>>& private_key =
                std::nullopt);

    ~SealedBox();

    std::vector<uint8_t> Encrypt(const std::string& plaintext);
    std::string Decrypt(const std::vector<uint8_t>& ciphertext);

  private:
    std::vector<uint8_t> _public_key;
    std::optional<std::vector<uint8_t>> _private_key;
  };

  std::vector<uint8_t> random_nonce();

} // end: namespace ifm3d

#endif // IFM3D_CRYPTO_CRYPTO_H
