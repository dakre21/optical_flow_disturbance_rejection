// container_reader.cpp
#include <boost/interprocess/exceptions.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <iostream>
#include <string>

int main() {
  using namespace boost::interprocess;

  const std::string shared_memory_name = "my_shared_memory";

  try {
    // Open existing shared memory object
    shared_memory_object shm(open_only, shared_memory_name.c_str(), read_only);

    // Map the whole shared memory in this process
    mapped_region region(shm, read_only);

    // Read data from shared memory
    std::string received_message(static_cast<char*>(region.get_address()));

    // Print the received message
    std::cout << "Received message from shared memory: " << received_message
              << std::endl;

  } catch (const interprocess_exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
