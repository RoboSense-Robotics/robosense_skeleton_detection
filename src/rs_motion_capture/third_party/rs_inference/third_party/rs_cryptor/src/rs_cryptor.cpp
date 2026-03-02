#include <string>
#include <cstring>
#include <vector>
#include <type_traits>
#include <iostream>
#include "rs_perception/rs_cryptor.h"

namespace robosense {
namespace perception {

class RsCryptorStaticBase {
protected:
    static const std::string m_base64_chars;
    static std::string m_key;
};

const std::string RsCryptorStaticBase::m_base64_chars =
"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string RsCryptorStaticBase::m_key = "robosense_cryptor";

class RsCryptorImpl : public RsCryptorStaticBase {
public:
    RsCryptorImpl() = default;

    void encrypt(const std::vector<char> &in, std::vector<char> &encrypt_out) {
        std::vector<char> tmp = in;
        do_xor(tmp);
        base64_encode(tmp, encrypt_out);
    }

    void decrypt(const std::vector<char> &in, std::vector<char> &decrypt_out) {
        base64_decode(in, decrypt_out);
        do_xor(decrypt_out);
    }

private:

    void do_xor(std::vector<char> &data) {
        for (size_t i = 0; i < data.size(); ++i) {
            data[i] ^= m_key.at(i % m_key.size());
        }
    }

    void base64_encode(const std::vector<char> &in, std::vector<char> &out) {
        out.clear();
        int len = static_cast<int>(in.size());
        int i = 0;
        int j = 0;
        uint8_t char_array_3[3];
        uint8_t char_array_4[4];
        auto in_ptr = in.data();
        while (len--) {
            char_array_3[i++] = *(in_ptr++);
            if (i == 3) {
                char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
                char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
                char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
                char_array_4[3] = char_array_3[2] & 0x3f;

                for (i = 0; i < 4; ++i) {
                    out.push_back(m_base64_chars.at(char_array_4[i]));
                }

                i = 0;
            }
        }

        if (i) {
            for (j = i; j < 3; ++j) {
                char_array_3[j] = '\0';
            }

            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);

            for (j = 0; j < i + 1; ++j) {
                out.push_back(m_base64_chars.at(char_array_4[j]));
            }

            while (i++ < 3) {
                out.push_back('=');
            }
        }
    }

    void base64_decode(const std::vector<char> &in, std::vector<char> &out_decrypt) {
        int in_len = static_cast<int>(in.size());
        out_decrypt.clear();
        int i = 0;
        int j = 0;
        int in_ = 0;
        uint8_t char_array_4[4], char_array_3[3];

        while (in_len-- && (in[in_] != '=') && is_base64(in[in_])) {
            char_array_4[i++] = in[in_];
            in_++;
            if (i == 4) {
                for (i = 0; i < 4; ++i) {
                    char_array_4[i] = static_cast<uint8_t>(m_base64_chars.find(char_array_4[i]));
                }

                char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
                char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
                char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

                for (i = 0; i < 3; ++i) {
                    out_decrypt.push_back(char_array_3[i]);
                }

                i = 0;
            }
        }

        if (i) {
            for (j = 0; j < i; ++j) {
                char_array_4[j] = static_cast<uint8_t>(m_base64_chars.find(char_array_4[j]));
            }

            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);

            for (j = 0; j < i - 1; ++j) {
                out_decrypt.push_back(char_array_3[j]);
            }
        }
    }

    bool is_base64(uint8_t c) {
        return (isalnum(c) || (c == '+') || (c == '/'));
    }

};


//========================================================================================================
RsCryptor::RsCryptor() {
    impl_ptr_ = new RsCryptorImpl;
}

RsCryptor::~RsCryptor() {
    delete impl_ptr_;
}

void RsCryptor::encrypt(const std::vector<char> &in, std::vector<char> &encrypt_out) {
    impl_ptr_->encrypt(in, encrypt_out);
}

void RsCryptor::decrypt(const std::vector<char> &in, std::vector<char> &decrypt_out) {
    impl_ptr_->decrypt(in, decrypt_out);
}

}
}

