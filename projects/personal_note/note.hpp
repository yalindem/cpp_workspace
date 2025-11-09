#ifndef NOTE_H
#define NOTE_H

#include <iostream>
#include <vector>
#include <algorithm>

class Note {
public:
    std::string header;
    std::string content;
    int id;

    Note(std::string h, std::string c, int i);
};

#endif // NOTE_H