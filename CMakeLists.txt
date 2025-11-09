cmake_minimum_required(VERSION 3.10)

# Projenin adını belirleyin
project(Project_test CXX)

# C++ standardını ayarlayın
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Projenin çalıştırılabilir dosyasını tanımlayın
add_executable(${PROJECT_NAME}
    main.cpp
    projects/personal_note/note.cpp
    projects/personal_note/note_manager.cpp
)

# Eğer header dosyalarınız projects/<proje_adi>/ dizininde ise, 
# derleyiciye bu dizini de bildirmemiz gerekir.
target_include_directories(${PROJECT_NAME} PUBLIC projects/personal_note)