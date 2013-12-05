remake_find_package(Threads)
if(NOT ${CMAKE_USE_PTHREADS_INIT})
  message(FATAL_ERROR "Missing POSIX thread support!")
endif(NOT ${CMAKE_USE_PTHREADS_INIT})

remake_include(../timer)
remake_add_library(
  thread
  LINK timer ${CMAKE_THREAD_LIBS_INIT}
)
remake_add_headers()
