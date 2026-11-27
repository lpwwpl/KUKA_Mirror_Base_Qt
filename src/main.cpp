#include "mainwindow.h"

#include <QApplication>
#include <QFont>
#include <QFontDatabase>
#include <QLocale>
#include <QTextCodec>

namespace {

QString U8(const char* text) { return QString::fromUtf8(text); }

void installChineseFont(QApplication& application) {
    const QStringList preferredFonts{
        U8("Microsoft YaHei UI"),
        U8("Microsoft YaHei"),
        U8("SimHei"),
        U8("Noto Sans CJK SC"),
        U8("WenQuanYi Micro Hei")
    };

    const QStringList installedFonts = QFontDatabase().families();
    for (const QString& family : preferredFonts) {
        if (installedFonts.contains(family, Qt::CaseInsensitive)) {
            QFont font = application.font();
            font.setFamily(family);
            application.setFont(font);
            return;
        }
    }
}

} // namespace

int main(int argc, char* argv[]) {
#if QT_VERSION >= QT_VERSION_CHECK(5, 6, 0)
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

    QApplication application(argc, argv);

    // Qt 5.12: make conversions that depend on the locale use UTF-8.
    QTextCodec* utf8 = QTextCodec::codecForName("UTF-8");
    if (utf8 != nullptr) {
        QTextCodec::setCodecForLocale(utf8);
    }

    // Keep decimal output compatible with KRL: always use '.' as decimal point.
    QLocale::setDefault(QLocale::c());
    installChineseFont(application);

    MainWindow window;
    window.show();
    return application.exec();
}
