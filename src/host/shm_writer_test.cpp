// host_writer.cpp
#include <boost/interprocess/exceptions.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <iostream>
#include <string>

int main() {
  using namespace boost::interprocess;

  const std::string shared_memory_name = "my_shared_memory";
  const size_t shared_memory_size = 1024;
  const std::string message_to_send = "Hello from the Host!";

  try {
    // Remove shared memory if it exists
    shared_memory_object::remove(shared_memory_name.c_str());

    // Create shared memory object
    shared_memory_object shm(create_only, shared_memory_name.c_str(),
                             read_write);

    // Set size of shared memory
    shm.truncate(shared_memory_size);

    // Map the whole shared memory in this process
    mapped_region region(shm, read_write);

    // Write data to shared memory
    std::memcpy(region.get_address(), message_to_send.c_str(),
                message_to_send.size() + 1);  // +1 for null terminator

    std::cout << "Message written to shared memory: " << message_to_send
              << std::endl;

    // Wait for the container to read the data (simplified wait)
    std::cout << "Press Enter to remove shared memory and exit..." << std::endl;
    std::cin.get();

    // Remove shared memory
    shared_memory_object::remove(shared_memory_name.c_str());
  } catch (const interprocess_exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    shared_memory_object::remove(shared_memory_name.c_str());
    return 1;
  }

  return 0;
}
