#include "note_manager.hpp"

NoteManager::NoteManager()
{

}

NoteManager::~NoteManager()
{
    
}

bool NoteManager::is_id_unique(int id) const
{
    for(const Note note : notes_)
    {
        if(note.id == id)
        {
            std::cerr << "Cannot add new note\n";
            return false;
        }
    }
    std::cout << "Adding note\n";
    return true;
}

void NoteManager::load_notes()
{
    std::cout << "file_name: " << FILE_NAME << "\n";
    std::ifstream file(FILE_NAME);
    std::string str;

    while(std::getline(file, str))
    {
        std::cout <<  "str: " << str << "\n";
        std::string delimiter = "|";
        std::vector<size_t> positions;
        size_t pos = str.find(delimiter, 0);
        while(pos != std::string::npos)
        {
            positions.push_back(pos);
            pos = str.find(delimiter, pos+1);
        }

        int id = std::stoi(str.substr(0, positions[0] + 1));
        std::string header = str.substr(positions[0] + 1, positions[1] - positions[0] - 1);
        std::string content = str.substr(positions[1] + 1);

        std::cout << "id: " << id << "\n";
        std::cout << header << "\n";
        std::cout << content << "\n";
        std::cout << "=======================================\n";

    }

    file.close();

}

void NoteManager::save_notes() const
{
    std::ofstream file(FILE_NAME); 

    if (file.is_open()) {
        for (const auto& note : notes_) {
            file << note.id << "|" 
                 << note.header << "|" 
                 << note.content << "\n";
        }
        file.close();
        std::cout << "Tum notlar basariyla dosyaya kaydedildi.\n";
    } 
    else 
    {
        std::cerr << "Hata: Kaydetme dosyasi acilamadi.\n";
    }
}

void NoteManager::add_note(const std::string& header, const std::string& content, const int id)
{
    if(this->is_id_unique(id))
    {
        this->notes_.emplace_back(Note(header, content, id));
    }
}

void NoteManager::delete_note(int id)
{
    int count = 0;
    for(const Note note : notes_)
    {
        if(note.id == id)
        {
            this->notes_.erase(this->notes_.begin() + count);
        }
        count++;
    }
}

void NoteManager::display_notes()
{
    load_notes();
}
