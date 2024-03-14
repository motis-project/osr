#include <cinttypes>
#include <cstring>

#include <unordered_map>

#include "osr/paged.h"

int run(std::uint8_t const*, size_t const);

#if defined(MAIN)
int main(int ac, char** av) {
  if (ac < 2) {
    std::cout << "usage: " << av[0] << " [crash-file]\n";
    return 0;
  }

  if (!std::filesystem::is_regular_file(av[1])) {
    std::cout << "file " << av[1] << " does not exist\n";
    return 0;
  }

  cista::mmap m{av[1], cista::mmap::protection::READ};
  return run(m.data(), m.size());
}
#else
extern "C" int LLVMFuzzerTestOneInput(uint8_t const* data, size_t size) {
  return run(data, size);
}
#endif

/**
 * Check hash_map<int, int>
 *
 *   Input bytes:
 *      0-3: key
 *      4-7: value size
 *        9: operation => less or equal 128 insert,
 *                     => greater 128 delete
 */
int run(std::uint8_t const* data, size_t const size) {
  auto const file = fmt::format("{}", std::tmpnam(nullptr));

  auto ec = std::error_code{};
  std::filesystem::remove_all(file, ec);

  auto str = std::string{};
  str.resize(4096U);
  std::generate(begin(str), end(str), [x = '\0']() mutable { return ++x; });

  auto ref = std::unordered_map<std::uint64_t, std::string_view>{};
  auto uut = osr::mmm<std::uint64_t>{file};

  for (auto bytes_to_process = size; bytes_to_process > 8;
       bytes_to_process -= 9, data += 9) {
    auto key = 0;
    auto value = 0;
    auto const insert = data[8] <= 128 ? true : false;

    std::memcpy(&key, data, sizeof(key));
    std::memcpy(&value, data + sizeof(key), sizeof(value));

    value %= 4096U;
    auto v = std::string_view{str}.substr(0, value);

    if (insert) {
      //      std::cerr << "INSERT " << key << " -> " << value << "\n";
      ref[key] = v;
      uut.put(key, v);
    } else {
      //      std::cerr << "ERASE " << key << "\n";
      ref.erase(key);
      uut.erase(key);
    }
  }

  assert(ref.size() == uut.size());
  for (auto const& [key, value] : ref) {
    auto const uut_value = uut.get(key);
    if (uut_value != value) {
      //      std::cout << "KEY=" << key << "\n";
      //      std::cout << "UUT VALUE SIZE: " << uut_value.size() << "\n";
      //      std::cout << "REF VALUE SIZE: " << value.size() << "\n";
      abort();
    }
  }

  return 0;
}
