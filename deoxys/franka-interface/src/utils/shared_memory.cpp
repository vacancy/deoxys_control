#include "utils/shared_memory.h"

static std::shared_ptr<SharedMemory> global_handler = nullptr;

std::shared_ptr<SharedMemory> createSharedMemory() {
  return std::make_shared<SharedMemory>();
}

std::shared_ptr<SharedMemory> getGlobalHandler() { return global_handler; }

void setGlobalHandler(std::shared_ptr<SharedMemory> handler) {
  global_handler = handler;
}
