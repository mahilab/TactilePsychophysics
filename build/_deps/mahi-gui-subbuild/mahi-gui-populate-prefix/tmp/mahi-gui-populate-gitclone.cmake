
if(NOT "C:/Git/TactilePsychophysics/build/_deps/mahi-gui-subbuild/mahi-gui-populate-prefix/src/mahi-gui-populate-stamp/mahi-gui-populate-gitinfo.txt" IS_NEWER_THAN "C:/Git/TactilePsychophysics/build/_deps/mahi-gui-subbuild/mahi-gui-populate-prefix/src/mahi-gui-populate-stamp/mahi-gui-populate-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: 'C:/Git/TactilePsychophysics/build/_deps/mahi-gui-subbuild/mahi-gui-populate-prefix/src/mahi-gui-populate-stamp/mahi-gui-populate-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "C:/Git/TactilePsychophysics/build/_deps/mahi-gui-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: 'C:/Git/TactilePsychophysics/build/_deps/mahi-gui-src'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe"  clone --no-checkout "https://github.com/mahilab/mahi-gui.git" "mahi-gui-src"
    WORKING_DIRECTORY "C:/Git/TactilePsychophysics/build/_deps"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/mahilab/mahi-gui.git'")
endif()

execute_process(
  COMMAND "C:/Program Files/Git/cmd/git.exe"  checkout master --
  WORKING_DIRECTORY "C:/Git/TactilePsychophysics/build/_deps/mahi-gui-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'master'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe"  submodule update --recursive --init 
    WORKING_DIRECTORY "C:/Git/TactilePsychophysics/build/_deps/mahi-gui-src"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: 'C:/Git/TactilePsychophysics/build/_deps/mahi-gui-src'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "C:/Git/TactilePsychophysics/build/_deps/mahi-gui-subbuild/mahi-gui-populate-prefix/src/mahi-gui-populate-stamp/mahi-gui-populate-gitinfo.txt"
    "C:/Git/TactilePsychophysics/build/_deps/mahi-gui-subbuild/mahi-gui-populate-prefix/src/mahi-gui-populate-stamp/mahi-gui-populate-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: 'C:/Git/TactilePsychophysics/build/_deps/mahi-gui-subbuild/mahi-gui-populate-prefix/src/mahi-gui-populate-stamp/mahi-gui-populate-gitclone-lastrun.txt'")
endif()

