#include <stdexcept>
#include <signal.h>
#include "Ncurses/InterfaceCLI.hpp"

namespace Leph {

InterfaceCLI::InterfaceCLI(const std::string& title) :
    _title(title),
    _sumParams(0),
    _sumMonitors(0),
    _selected(0),
    _paramsDelta(0.2)
{
    //Ncurses initialization
    initscr();
    //Enable special input keys
    keypad(stdscr, TRUE);
    //Disable input echo
    noecho();
    //Disable input buffering
    //(and catches Ctrl-Z and Ctrl-C)
    raw();
    //Enable non-blocking input reading
    nodelay(stdscr, TRUE);
    //Hide cursor
    curs_set(0);

    //Draw first main window
    refresh();
    
    //Creating ncurses windows
    _titleWin = newwin(9, 38, 1, 1);
    _statusWin = newwin(5, 38, 11, 1);
    _paramsWin = NULL;
    _monitorsWin = NULL;
    
    //Draw first windows
    drawTitleWin();
    drawStatusWin();
}
        
InterfaceCLI::~InterfaceCLI()
{
    //Free ncurses windows
    if (_paramsWin != NULL) {
        delwin(_paramsWin);
    }
    if (_monitorsWin != NULL) {
        delwin(_monitorsWin);
    }
    delwin(_titleWin);
    endwin();
}
        
void InterfaceCLI::addParameters(const std::string& sectionName,
    VectorLabel& vector)
{
    if (_parameters.count(sectionName) > 0) {
        throw std::logic_error("InterfaceCLI already use section");
    }
    _parameters[sectionName] = &vector;
    _sumParams += vector.size();

    if (_paramsWin != NULL) {
        delwin(_paramsWin);
    }
    _paramsWin = newwin(_sumParams+4+_parameters.size(), 40, 1, 40);

    drawParamsWin(); 
}

void InterfaceCLI::addMonitors(const std::string& sectionName,
    VectorLabel& vector)
{
    if (_monitors.count(sectionName) > 0) {
        throw std::logic_error("InterfaceCLI already use section");
    }
    _monitors[sectionName] = &vector;
    _sumMonitors += vector.size();

    if (_monitorsWin != NULL) {
        delwin(_monitorsWin);
    }
    _monitorsWin = newwin(_sumMonitors+4+_monitors.size(), 38, 17, 1);

    drawMonitorsWin(); 
}
        
void InterfaceCLI::addBinding(int key, const std::string& helpMsg,
    std::function<void()> handler)
{
    _userBindings.push_back({key, handler, helpMsg});

    delwin(_titleWin);
    delwin(_statusWin);
    _titleWin = newwin(9 + _userBindings.size(), 38, 1, 1);
    _statusWin = newwin(5 + _userStatus.size(), 38, 11 + _userBindings.size(), 1);

    if (_monitorsWin != NULL) {
        delwin(_monitorsWin);
        _monitorsWin = newwin(_sumMonitors+4+_monitors.size(), 38, 
            17 + _userBindings.size() + _userStatus.size(), 1);
    }

    clear();
    refresh();
    drawTitleWin();
    drawStatusWin();
    drawMonitorsWin();
    drawParamsWin();
}
        
void InterfaceCLI::addStatus(std::string& line)
{
    _userStatus.push_back(&line);
    
    delwin(_statusWin);
    _statusWin = newwin(5 + _userStatus.size(), 38, 11 + _userBindings.size(), 1);

    if (_monitorsWin != NULL) {
        delwin(_monitorsWin);
        _monitorsWin = newwin(_sumMonitors+4+_monitors.size(), 38, 
            17 + _userBindings.size() + _userStatus.size(), 1);
    }

    clear();
    refresh();
    drawTitleWin();
    drawStatusWin();
    drawMonitorsWin();
    drawParamsWin();
}

bool InterfaceCLI::tick(bool updateAll)
{
    int input = getch();
    if (input == 'q') {
        return false;
    } else if (input == KEY_DOWN) {
        if (_selected < _sumParams-1) {
            _selected++;
            drawParamsWin();
        }
    } else if (input == KEY_UP) {
        if (_selected > 0) {
            _selected--;
            drawParamsWin();
        }
    } else if (input == '+') {
        _paramsDelta *= 2.0;
        drawStatusWin();
    } else if (input == '-') {
        _paramsDelta /= 2.0;
        drawStatusWin();
    } else if (input == KEY_LEFT) {
        getSelected() -= _paramsDelta;
        drawParamsWin();
    } else if (input == KEY_RIGHT) {
        getSelected() += _paramsDelta;
        drawParamsWin();
    } else if (input == '0') {
        getSelected() = 0.0;
        drawParamsWin();
    }
    for (size_t i=0;i<_userBindings.size();i++) {
        if (input == _userBindings[i].key) {
            _userBindings[i].handler();
        }
    }

    if (updateAll) {
        drawStatusWin();
        drawMonitorsWin();
    }

    return true;
}
        
void InterfaceCLI::drawTitleWin()
{
    wborder(_titleWin, 
        ACS_VLINE, ACS_VLINE, ACS_HLINE, ACS_HLINE, 
        ACS_ULCORNER, ACS_URCORNER, ACS_LLCORNER, ACS_LRCORNER);
    wattrset(_titleWin, A_BOLD | A_UNDERLINE);
    mvwprintw(_titleWin, 1, 1, _title.c_str());
    wattrset(_titleWin, A_NORMAL);
    mvwprintw(_titleWin, 3, 1, "q:          Quit");
    mvwprintw(_titleWin, 4, 1, "up/down:    Select parameter");
    mvwprintw(_titleWin, 5, 1, "left/right: Decr/Incr parameter");
    mvwprintw(_titleWin, 6, 1, "+/-:        Incr/Decr delta param");
    mvwprintw(_titleWin, 7, 1, "0:          Set parameter to 0");
    for (size_t i=0;i<_userBindings.size();i++) {
        mvwprintw(_titleWin, 8+i, 1, "%c: %s", 
            _userBindings[i].key, _userBindings[i].help.c_str());
    }
    wrefresh(_titleWin);
}

void InterfaceCLI::drawStatusWin()
{
    wclear(_statusWin);
    wborder(_statusWin, 
        ACS_VLINE, ACS_VLINE, ACS_HLINE, ACS_HLINE, 
        ACS_ULCORNER, ACS_URCORNER, ACS_LLCORNER, ACS_LRCORNER);
    wattrset(_statusWin, A_BOLD | A_UNDERLINE);
    mvwprintw(_statusWin, 1, 1, "Status");
    wattrset(_statusWin, A_NORMAL);
    mvwprintw(_statusWin, 3, 1, "Delta (parameter)       %0.4f", _paramsDelta);
    for (size_t i=0;i<_userStatus.size();i++) {
        mvwprintw(_statusWin, 4+i, 1, "%s", _userStatus[i]->c_str());
    }
    wrefresh(_statusWin);
}

void InterfaceCLI::drawParamsWin()
{
    wclear(_paramsWin);
    wborder(_paramsWin, '|', '|', '-', '-', '#', '#', '#', '#');
    wattrset(_paramsWin, A_BOLD | A_UNDERLINE);
    mvwprintw(_paramsWin, 1, 1, "Parameters");
    wattrset(_paramsWin, A_NORMAL);
    
    size_t line = 3;
    size_t sumIndex = 0;
    for (auto vector : _parameters) {
        wattrset(_paramsWin, A_BOLD);
        mvwprintw(_paramsWin, line, 1, vector.first.c_str());
        wattrset(_paramsWin, A_NORMAL);
        line ++;
        for (size_t i=0;i<vector.second->size();i++) {
            if (_selected == sumIndex) {
                wattrset(_paramsWin, A_STANDOUT);
                std::string label = "- " + vector.second->getLabel(i);
                mvwprintw(_paramsWin, line, 2, label.c_str());
                mvwprintw(_paramsWin, line, 25, "%.4f", (*vector.second)(i));
                wattrset(_paramsWin, A_NORMAL);
            } else {
                std::string label = "- " + vector.second->getLabel(i);
                mvwprintw(_paramsWin, line, 2, label.c_str());
                mvwprintw(_paramsWin, line, 25, "%.4f", (*vector.second)(i));
            }
            sumIndex++;
            line++;
        }
    }

    wrefresh(_paramsWin);
}

void InterfaceCLI::drawMonitorsWin()
{
    wclear(_monitorsWin);
    wborder(_monitorsWin, '|', '|', '-', '-', '#', '#', '#', '#');
    wattrset(_monitorsWin, A_BOLD | A_UNDERLINE);
    mvwprintw(_monitorsWin, 1, 1, "Monitors");
    wattrset(_monitorsWin, A_NORMAL);
    
    size_t line = 3;
    for (auto vector : _monitors) {
        wattrset(_monitorsWin, A_BOLD);
        mvwprintw(_monitorsWin, line, 1, vector.first.c_str());
        wattrset(_monitorsWin, A_NORMAL);
        line += 1;
        for (size_t i=0;i<vector.second->size();i++) {
            std::string label = "- " + vector.second->getLabel(i);
            mvwprintw(_monitorsWin, line, 2, label.c_str());
            mvwprintw(_monitorsWin, line, 25, "%.4f", (*vector.second)(i));
            line++;
        }
    }

    wrefresh(_monitorsWin);
}
        
double& InterfaceCLI::getSelected()
{
    size_t sumIndex = 0;
    for (auto vector : _parameters) {
        for (size_t i=0;i<vector.second->size();i++) {
            if (_selected == sumIndex) {
                return (*vector.second)(i);
            }
            sumIndex++;
        }
    }

    throw std::logic_error("InterfaceCLI error selection");
}
        
}

