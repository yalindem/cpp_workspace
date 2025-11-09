
#include "note.hpp"
#include "note_manager.hpp"

Note::Note(std::string h, std::string c, int i) : 
        header(std::move(h)), content(std::move(c)), id(i) 
{

}

