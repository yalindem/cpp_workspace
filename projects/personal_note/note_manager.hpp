// manager.hpp
#include <vector>
#include <string>
#include <fstream>

#include "note.hpp"


class NoteManager {
private:
    std::vector<Note> notes_; // Notların tek merkezi listesi
    const std::string FILE_NAME = "/home/yalin/workspace/tutorials/cpp_workspace/projects/personal_note/notlar.txt"; // Kalıcılık için dosya adı

    bool is_id_unique(int id) const;
    void load_notes();
    void save_notes() const;

    std::string notes_str_;

public:
    NoteManager();
    ~NoteManager();
    
    void add_note(const std::string& header, const std::string& content, const int id); 
    void delete_note(int id);
    void display_notes();
};