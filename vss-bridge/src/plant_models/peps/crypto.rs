//! Cryptographic primitives for PEPS plant model simulation.
//!
//! The plant model simulates the crypto operations that real key fobs, BLE
//! phones, and NFC cards perform:
//!
//! - **AES-128 challenge-response:** Vehicle sends a 128-bit random nonce via
//!   LF (or BLE / NFC); the device encrypts it with the shared secret and
//!   returns the ciphertext. The vehicle-side feature logic verifies.
//!
//! - **AES-CMAC (RFC 4493):** RF messages are authenticated with a truncated
//!   AES-CMAC tag. The plant model generates the MAC; the vehicle-side RKE
//!   feature logic verifies it before acting on the command.
//!
//! - **Rolling code:** RF fob button presses include an unencrypted rolling
//!   code counter plus a MAC covering fob_id + action + counter. The vehicle-
//!   side RKE feature logic handles validation and resync windowing.
//!
//! This module uses software AES-128-ECB for simulation. In a real system
//! the key fob has a hardware AES engine and the vehicle uses an HSM.

/// 128-bit (16-byte) shared secret key.
pub type SharedSecret = [u8; 16];

/// 128-bit (16-byte) challenge nonce.
pub type Challenge = [u8; 16];

/// 128-bit (16-byte) encrypted response.
pub type ChallengeResponse = [u8; 16];

/// Encrypt a 16-byte block with AES-128-ECB (single block, no padding needed).
///
/// In production this runs on the fob/phone's hardware AES engine.
/// For simulation we use a simple software implementation.
pub fn aes128_encrypt_block(key: &SharedSecret, plaintext: &[u8; 16]) -> [u8; 16] {
    // Software AES-128-ECB: expand key, then encrypt one 16-byte block.
    let round_keys = aes128_key_expansion(key);
    let mut state = *plaintext;
    aes128_encrypt_state(&mut state, &round_keys);
    state
}

/// Compute a challenge response: AES-128(shared_secret, nonce).
pub fn compute_challenge_response(key: &SharedSecret, nonce: &Challenge) -> ChallengeResponse {
    aes128_encrypt_block(key, nonce)
}

/// Encrypt a rolling code counter for RF transmission.
/// The counter is zero-padded to 16 bytes, then AES-128 encrypted.
pub fn encrypt_rolling_code(key: &SharedSecret, counter: u32) -> [u8; 16] {
    let mut block = [0u8; 16];
    block[0..4].copy_from_slice(&counter.to_le_bytes());
    aes128_encrypt_block(key, &block)
}

/// Truncated AES-CMAC tag length used in RF messages (7 bytes = 56 bits).
///
/// 56 bits provides adequate security for short-range RF with a rolling code;
/// accepted by the RKE receiver provided the rolling code and fob ID also match.
pub const RF_MAC_LEN: usize = 7;

/// Truncated AES-CMAC tag for an RF message.
pub type RfMac = [u8; RF_MAC_LEN];

/// Compute AES-CMAC (RFC 4493) over `message` using `key`, then truncate to
/// `RF_MAC_LEN` bytes.
///
/// The CMAC covers the full RF message payload — fob ID, action byte, and
/// rolling code — so the MAC binds all three fields together.
///
/// # Algorithm (RFC 4493 §2.4)
/// 1. Derive subkeys K1, K2 from AES(key, 0^128).
/// 2. Split message into 16-byte blocks; pad last block if needed.
/// 3. XOR last block with K1 (complete) or K2 (padded).
/// 4. CBC-MAC: X₀ = 0^128, Xᵢ = AES(key, Xᵢ₋₁ ⊕ Mᵢ).
/// 5. Return first `RF_MAC_LEN` bytes of the final block.
pub fn aes_cmac_truncated(key: &SharedSecret, message: &[u8]) -> RfMac {
    let (k1, k2) = cmac_generate_subkeys(key);

    // Split message into 16-byte blocks.
    let msg_len = message.len();
    let (n, last_complete) = if msg_len == 0 {
        (1usize, false)
    } else {
        let n = msg_len.div_ceil(16);
        let last_complete = msg_len.is_multiple_of(16);
        (n, last_complete)
    };

    // Build the (potentially padded) last block.
    let mut last_block = [0u8; 16];
    if msg_len == 0 {
        // Empty message: pad with 0x80 00 .. 00 and use K2.
        last_block[0] = 0x80;
    } else {
        let last_start = (n - 1) * 16;
        let last_bytes = &message[last_start..];
        last_block[..last_bytes.len()].copy_from_slice(last_bytes);
        if !last_complete {
            // Apply ISO/IEC 7816-4 padding: append 0x80 after the last data byte.
            last_block[last_bytes.len()] = 0x80;
        }
    }

    // XOR last block with appropriate subkey.
    let subkey = if last_complete { &k1 } else { &k2 };
    for i in 0..16 {
        last_block[i] ^= subkey[i];
    }

    // CBC-MAC over blocks 0..n-1 using last_block as block n-1.
    let round_keys = aes128_key_expansion(key);
    let mut x = [0u8; 16];

    // All blocks except the last.
    for block_idx in 0..n.saturating_sub(1) {
        let start = block_idx * 16;
        let end = start + 16;
        let block = &message[start..end];
        for i in 0..16 {
            x[i] ^= block[i];
        }
        aes128_encrypt_state(&mut x, &round_keys);
    }

    // Final block.
    for i in 0..16 {
        x[i] ^= last_block[i];
    }
    aes128_encrypt_state(&mut x, &round_keys);

    // Truncate to RF_MAC_LEN.
    let mut tag = [0u8; RF_MAC_LEN];
    tag.copy_from_slice(&x[..RF_MAC_LEN]);
    tag
}

/// Verify an AES-CMAC tag (constant-time byte comparison).
///
/// Returns `true` if the provided `tag` matches the MAC computed over
/// `message` with `key`.
pub fn aes_cmac_verify(key: &SharedSecret, message: &[u8], tag: &RfMac) -> bool {
    let expected = aes_cmac_truncated(key, message);
    // Constant-time comparison to resist timing attacks.
    let mut diff = 0u8;
    for i in 0..RF_MAC_LEN {
        diff |= expected[i] ^ tag[i];
    }
    diff == 0
}

/// Derive CMAC subkeys K1 and K2 from the cipher key (RFC 4493 §2.3).
fn cmac_generate_subkeys(key: &SharedSecret) -> ([u8; 16], [u8; 16]) {
    // L = AES(key, 0^128)
    let round_keys = aes128_key_expansion(key);
    let mut l = [0u8; 16];
    aes128_encrypt_state(&mut l, &round_keys);

    // Rb for AES-128: 0x00..00 0x87
    const RB: u8 = 0x87;

    // K1 = (MSB(L) == 0) ? L << 1 : (L << 1) XOR Rb
    let k1 = {
        let msb = l[0] & 0x80;
        let mut k = left_shift_one(&l);
        if msb != 0 {
            k[15] ^= RB;
        }
        k
    };

    // K2 = (MSB(K1) == 0) ? K1 << 1 : (K1 << 1) XOR Rb
    let k2 = {
        let msb = k1[0] & 0x80;
        let mut k = left_shift_one(&k1);
        if msb != 0 {
            k[15] ^= RB;
        }
        k
    };

    (k1, k2)
}

/// Left-shift a 16-byte block by one bit (MSB first).
fn left_shift_one(block: &[u8; 16]) -> [u8; 16] {
    let mut out = [0u8; 16];
    let mut carry = 0u8;
    for i in (0..16).rev() {
        out[i] = (block[i] << 1) | carry;
        carry = (block[i] >> 7) & 1;
    }
    out
}

/// Generate a random 128-bit nonce (for challenges).
/// In tests with deterministic seeds this can be replaced.
pub fn random_nonce() -> Challenge {
    let mut nonce = [0u8; 16];
    // Use a simple PRNG seeded from thread_rng for simulation purposes.
    // Not cryptographically strong — this is a plant model, not production crypto.
    for byte in &mut nonce {
        *byte = fastrand::u8(..);
    }
    nonce
}

// ---------------------------------------------------------------------------
// AES-128 software implementation (single-block ECB)
// ---------------------------------------------------------------------------
// This is a standard FIPS-197 implementation for simulation only.
// Production systems use hardware AES (HSM / fob crypto engine).

/// AES S-box lookup table.
const SBOX: [u8; 256] = [
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
];

/// AES round constants.
const RCON: [u8; 10] = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36];

/// Expand a 128-bit key into 11 round keys (44 32-bit words).
fn aes128_key_expansion(key: &[u8; 16]) -> [[u8; 16]; 11] {
    let mut w = [0u32; 44];

    // First 4 words are the key itself.
    for i in 0..4 {
        w[i] = u32::from_be_bytes([key[4 * i], key[4 * i + 1], key[4 * i + 2], key[4 * i + 3]]);
    }

    for i in 4..44 {
        let mut temp = w[i - 1];
        if i % 4 == 0 {
            // RotWord + SubWord + Rcon
            temp = temp.rotate_left(8);
            let bytes = temp.to_be_bytes();
            temp = u32::from_be_bytes([
                SBOX[bytes[0] as usize],
                SBOX[bytes[1] as usize],
                SBOX[bytes[2] as usize],
                SBOX[bytes[3] as usize],
            ]);
            temp ^= (RCON[i / 4 - 1] as u32) << 24;
        }
        w[i] = w[i - 4] ^ temp;
    }

    let mut round_keys = [[0u8; 16]; 11];
    for (r, round_key) in round_keys.iter_mut().enumerate() {
        for j in 0..4 {
            let bytes = w[r * 4 + j].to_be_bytes();
            round_key[4 * j..4 * j + 4].copy_from_slice(&bytes);
        }
    }
    round_keys
}

/// Encrypt a 16-byte state in-place using expanded round keys.
fn aes128_encrypt_state(state: &mut [u8; 16], round_keys: &[[u8; 16]; 11]) {
    // Initial round key addition
    xor_block(state, &round_keys[0]);

    // Rounds 1..9
    for round_key in &round_keys[1..10] {
        sub_bytes(state);
        shift_rows(state);
        mix_columns(state);
        xor_block(state, round_key);
    }

    // Final round (no MixColumns)
    sub_bytes(state);
    shift_rows(state);
    xor_block(state, &round_keys[10]);
}

fn xor_block(state: &mut [u8; 16], key: &[u8; 16]) {
    for i in 0..16 {
        state[i] ^= key[i];
    }
}

fn sub_bytes(state: &mut [u8; 16]) {
    for byte in state.iter_mut() {
        *byte = SBOX[*byte as usize];
    }
}

fn shift_rows(state: &mut [u8; 16]) {
    // AES state is column-major: state[row + 4*col]
    // Row 0: no shift
    // Row 1: shift left by 1
    let t = state[1];
    state[1] = state[5];
    state[5] = state[9];
    state[9] = state[13];
    state[13] = t;
    // Row 2: shift left by 2
    let (t0, t1) = (state[2], state[6]);
    state[2] = state[10];
    state[6] = state[14];
    state[10] = t0;
    state[14] = t1;
    // Row 3: shift left by 3 (= right by 1)
    // Row 3 indices are 3, 7, 11, 15
    let t = state[15];
    state[15] = state[11];
    state[11] = state[7];
    state[7] = state[3];
    state[3] = t;
}

/// GF(2^8) multiplication by 2.
fn xtime(a: u8) -> u8 {
    let shifted = (a as u16) << 1;
    let result = shifted ^ (if a & 0x80 != 0 { 0x1b } else { 0 });
    result as u8
}

fn mix_columns(state: &mut [u8; 16]) {
    for col in 0..4 {
        let i = col * 4;
        let (s0, s1, s2, s3) = (state[i], state[i + 1], state[i + 2], state[i + 3]);
        let t = s0 ^ s1 ^ s2 ^ s3;
        state[i] = s0 ^ xtime(s0 ^ s1) ^ t;
        state[i + 1] = s1 ^ xtime(s1 ^ s2) ^ t;
        state[i + 2] = s2 ^ xtime(s2 ^ s3) ^ t;
        state[i + 3] = s3 ^ xtime(s3 ^ s0) ^ t;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// NIST FIPS-197 Appendix B test vector.
    #[test]
    fn aes128_nist_test_vector() {
        let key: [u8; 16] = [
            0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf,
            0x4f, 0x3c,
        ];
        let plaintext: [u8; 16] = [
            0x32, 0x43, 0xf6, 0xa8, 0x88, 0x5a, 0x30, 0x8d, 0x31, 0x31, 0x98, 0xa2, 0xe0, 0x37,
            0x07, 0x34,
        ];
        let expected: [u8; 16] = [
            0x39, 0x25, 0x84, 0x1d, 0x02, 0xdc, 0x09, 0xfb, 0xdc, 0x11, 0x85, 0x97, 0x19, 0x6a,
            0x0b, 0x32,
        ];
        let result = aes128_encrypt_block(&key, &plaintext);
        assert_eq!(result, expected, "NIST AES-128 test vector failed");
    }

    #[test]
    fn challenge_response_deterministic() {
        let key: SharedSecret = [0xAA; 16];
        let nonce: Challenge = [0x55; 16];
        let r1 = compute_challenge_response(&key, &nonce);
        let r2 = compute_challenge_response(&key, &nonce);
        assert_eq!(r1, r2, "same key + nonce must produce same response");
    }

    #[test]
    fn different_keys_produce_different_responses() {
        let nonce: Challenge = [0x42; 16];
        let key_a: SharedSecret = [0x01; 16];
        let key_b: SharedSecret = [0x02; 16];
        let ra = compute_challenge_response(&key_a, &nonce);
        let rb = compute_challenge_response(&key_b, &nonce);
        assert_ne!(ra, rb, "different keys must produce different responses");
    }

    #[test]
    fn rolling_code_encryption() {
        let key: SharedSecret = [0xBB; 16];
        let enc_1 = encrypt_rolling_code(&key, 1);
        let enc_2 = encrypt_rolling_code(&key, 2);
        assert_ne!(
            enc_1, enc_2,
            "different counters must produce different ciphertext"
        );

        // Same counter = same ciphertext (deterministic)
        let enc_1b = encrypt_rolling_code(&key, 1);
        assert_eq!(enc_1, enc_1b);
    }

    #[test]
    fn random_nonce_produces_16_bytes() {
        let n = random_nonce();
        assert_eq!(n.len(), 16);
        // Very unlikely all zeros from random
        assert!(n.iter().any(|&b| b != 0), "nonce should not be all zeros");
    }

    // -----------------------------------------------------------------------
    // AES-CMAC tests (RFC 4493 §D test vectors)
    // -----------------------------------------------------------------------
    // Key used by all RFC 4493 test cases:
    // 2b7e1516 28aed2a6 abf71588 09cf4f3c
    fn rfc4493_key() -> SharedSecret {
        [
            0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf,
            0x4f, 0x3c,
        ]
    }

    /// RFC 4493 §D Example 1: empty message.
    /// Full CMAC (16 bytes) = bb1d6929 e9593728 7fa37d12 9b756746
    #[test]
    fn cmac_rfc4493_example1_empty() {
        let key = rfc4493_key();
        let full_mac = aes_cmac_full(&key, &[]);
        let expected: [u8; 16] = [
            0xbb, 0x1d, 0x69, 0x29, 0xe9, 0x59, 0x37, 0x28, 0x7f, 0xa3, 0x7d, 0x12, 0x9b, 0x75,
            0x67, 0x46,
        ];
        assert_eq!(
            full_mac, expected,
            "RFC 4493 Example 1 (empty message) failed"
        );
    }

    /// RFC 4493 §D Example 2: 16-byte message.
    /// Message = 6bc1bee2 2e409f96 e93d7e11 7393172a
    /// Full CMAC = 070a16b4 6b4d4144 f79bdd9d d04a287c
    #[test]
    fn cmac_rfc4493_example2_one_block() {
        let key = rfc4493_key();
        let msg: [u8; 16] = [
            0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93,
            0x17, 0x2a,
        ];
        let full_mac = aes_cmac_full(&key, &msg);
        let expected: [u8; 16] = [
            0x07, 0x0a, 0x16, 0xb4, 0x6b, 0x4d, 0x41, 0x44, 0xf7, 0x9b, 0xdd, 0x9d, 0xd0, 0x4a,
            0x28, 0x7c,
        ];
        assert_eq!(
            full_mac, expected,
            "RFC 4493 Example 2 (16-byte message) failed"
        );
    }

    /// RFC 4493 §D Example 3: 40-byte message (2.5 blocks).
    /// Message = 6bc1bee2 2e409f96 e93d7e11 7393172a
    ///           ae2d8a57 1e03ac9c 9eb76fac 45af8e51
    ///           30c81c46 a35ce411
    /// Full CMAC = dfa66747 de9ae630 30ca3261 1497c827
    #[test]
    fn cmac_rfc4493_example3_partial_last_block() {
        let key = rfc4493_key();
        let msg: [u8; 40] = [
            0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93,
            0x17, 0x2a, 0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7, 0x6f, 0xac,
            0x45, 0xaf, 0x8e, 0x51, 0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11,
        ];
        let full_mac = aes_cmac_full(&key, &msg);
        let expected: [u8; 16] = [
            0xdf, 0xa6, 0x67, 0x47, 0xde, 0x9a, 0xe6, 0x30, 0x30, 0xca, 0x32, 0x61, 0x14, 0x97,
            0xc8, 0x27,
        ];
        assert_eq!(
            full_mac, expected,
            "RFC 4493 Example 3 (40-byte message) failed"
        );
    }

    /// RFC 4493 §D Example 4: 64-byte message (exactly 4 blocks).
    /// Full CMAC = 51f0bebf 7e3b9d92 fc497417 79363cfe
    #[test]
    fn cmac_rfc4493_example4_four_blocks() {
        let key = rfc4493_key();
        let msg: [u8; 64] = [
            0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93,
            0x17, 0x2a, 0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7, 0x6f, 0xac,
            0x45, 0xaf, 0x8e, 0x51, 0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11, 0xe5, 0xfb,
            0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef, 0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17,
            0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10,
        ];
        let full_mac = aes_cmac_full(&key, &msg);
        let expected: [u8; 16] = [
            0x51, 0xf0, 0xbe, 0xbf, 0x7e, 0x3b, 0x9d, 0x92, 0xfc, 0x49, 0x74, 0x17, 0x79, 0x36,
            0x3c, 0xfe,
        ];
        assert_eq!(
            full_mac, expected,
            "RFC 4493 Example 4 (64-byte message) failed"
        );
    }

    /// Truncated CMAC: first 7 bytes of example 4 must match.
    #[test]
    fn cmac_truncated_matches_full_prefix() {
        let key = rfc4493_key();
        let msg: [u8; 64] = [
            0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93,
            0x17, 0x2a, 0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7, 0x6f, 0xac,
            0x45, 0xaf, 0x8e, 0x51, 0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11, 0xe5, 0xfb,
            0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef, 0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17,
            0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10,
        ];
        let full = aes_cmac_full(&key, &msg);
        let truncated = aes_cmac_truncated(&key, &msg);
        assert_eq!(
            truncated,
            full[..RF_MAC_LEN],
            "truncated CMAC must be the first RF_MAC_LEN bytes of the full CMAC"
        );
    }

    /// Verify: correct tag passes, flipped bit fails.
    #[test]
    fn cmac_verify_accept_reject() {
        let key = rfc4493_key();
        let msg = b"fob0001\x02\x00\x00\x04\x00"; // sample RF payload
        let tag = aes_cmac_truncated(&key, msg);
        assert!(aes_cmac_verify(&key, msg, &tag), "valid tag must verify");

        let mut bad_tag = tag;
        bad_tag[3] ^= 0x01;
        assert!(
            !aes_cmac_verify(&key, msg, &bad_tag),
            "tampered tag must not verify"
        );
    }

    /// Different keys produce different MACs.
    #[test]
    fn cmac_key_sensitivity() {
        let key_a: SharedSecret = [0x01; 16];
        let key_b: SharedSecret = [0x02; 16];
        let msg = b"same message";
        let mac_a = aes_cmac_truncated(&key_a, msg);
        let mac_b = aes_cmac_truncated(&key_b, msg);
        assert_ne!(mac_a, mac_b, "different keys must produce different MACs");
    }

    /// Different messages produce different MACs (same key).
    #[test]
    fn cmac_message_sensitivity() {
        let key = rfc4493_key();
        let mac_a = aes_cmac_truncated(&key, b"message one");
        let mac_b = aes_cmac_truncated(&key, b"message two");
        assert_ne!(
            mac_a, mac_b,
            "different messages must produce different MACs"
        );
    }
}

// Helper exposed only for tests: full 16-byte AES-CMAC (no truncation).
#[cfg(test)]
fn aes_cmac_full(key: &SharedSecret, message: &[u8]) -> [u8; 16] {
    let (k1, k2) = cmac_generate_subkeys(key);

    let msg_len = message.len();
    let (n, last_complete) = if msg_len == 0 {
        (1usize, false)
    } else {
        let n = msg_len.div_ceil(16);
        let last_complete = msg_len.is_multiple_of(16);
        (n, last_complete)
    };

    let mut last_block = [0u8; 16];
    if msg_len == 0 {
        last_block[0] = 0x80;
    } else {
        let last_start = (n - 1) * 16;
        let last_bytes = &message[last_start..];
        last_block[..last_bytes.len()].copy_from_slice(last_bytes);
        if !last_complete {
            last_block[last_bytes.len()] = 0x80;
        }
    }

    let subkey = if last_complete { &k1 } else { &k2 };
    for i in 0..16 {
        last_block[i] ^= subkey[i];
    }

    let round_keys = aes128_key_expansion(key);
    let mut x = [0u8; 16];

    for block_idx in 0..n.saturating_sub(1) {
        let start = block_idx * 16;
        let end = start + 16;
        let block = &message[start..end];
        for i in 0..16 {
            x[i] ^= block[i];
        }
        aes128_encrypt_state(&mut x, &round_keys);
    }

    for i in 0..16 {
        x[i] ^= last_block[i];
    }
    aes128_encrypt_state(&mut x, &round_keys);
    x
}
