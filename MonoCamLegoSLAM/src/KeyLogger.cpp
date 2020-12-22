#include <KeyLogger.hpp>

// Addapted from https://gist.github.com/whyrusleeping/3983293

#include <termios.h>
#include <unistd.h>
#include <stdlib.h>

void RestoreKeyboardBlocking(termios *initial_settings) {
	tcsetattr(0, TCSANOW, initial_settings);
}

void SetKeyboardNonBlock(struct termios *initial_settings) {
    auto new_settings = termios{};
    tcgetattr(0, initial_settings);

    new_settings = *initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;

    tcsetattr(0, TCSANOW, &new_settings);
}

char getCharAlt() {
    char buff[2] = {};
    int l = read(STDIN_FILENO, buff, 1);
    if (l > 0) {
        return buff[0];
    }
    return 0;
}

void logKeys(std::function<void(Key)> callback) {
    auto term_settings = termios{};
    auto c = char{};

    SetKeyboardNonBlock(&term_settings);

    auto done = false;
    while (!done) {
        c = getCharAlt();
        if (c == 0) {
            continue;
        }
        switch (c) {
        case 'q':
        case 'Q':
            done = true;
            break;
        case 'w':
        case 'W':
            callback(Key::W);
            break;
        case 'a':
        case 'A':
            callback(Key::A);
            break;
        case 's':
        case 'S':
            callback(Key::S);
            break;
        case 'd':
        case 'D':
            callback(Key::D);
            break;
        }
        // if (c > 0) {
        //     std::cout << "\nRead " << static_cast<int>(c) << "\n";
        // }
    }

    //Not restoring the keyboard settings causes the input from the terminal to not work right
    RestoreKeyboardBlocking(&term_settings);
}