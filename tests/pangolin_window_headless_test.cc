#include "ui/pangolin_window.h"

int main() {
    lightning::ui::PangolinWindow window;
    if (!window.Init()) {
        return 1;
    }

    window.Quit();
    return window.ShouldQuit() ? 2 : 0;
}
