#ifndef RS_PERCEPTION_CRYPTOR_RS_CRYPTOR_H
#define RS_PERCEPTION_CRYPTOR_RS_CRYPTOR_H

#include <ios>
#include <vector>

namespace robosense {
namespace perception {

class RsCryptorImpl;

class RsCryptor {
 public:
  RsCryptor();

  ~RsCryptor();

  void encrypt(const std::vector<char> &in, std::vector<char> &encrypt_out);

  void decrypt(const std::vector<char> &in, std::vector<char> &encrypt_out);

 private:
  RsCryptorImpl *impl_ptr_;
};

}  // namespace perception
}  // namespace robosense

#endif  // RS_PERCEPTION_CRYPTOR_RS_CRYPTOR_H
