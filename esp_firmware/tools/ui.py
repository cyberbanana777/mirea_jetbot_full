"""Красивый вывод для mtcal.py / steptest.py через rich (pip install rich).
Если rich не установлен, всё печатается обычным текстом - скрипты работают так же."""
try:
    from rich.console import Console
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
    HAVE_RICH = True
    console = Console()
except ImportError:                                     # без rich - простой текст
    HAVE_RICH = False
    console = None


def _plain(c):
    return c[0] if isinstance(c, tuple) else str(c)


def say(msg, style=None):
    """Одна строка текста. style - стиль rich, например 'dim', 'bold cyan'."""
    if HAVE_RICH:
        console.print(Text(msg, style=style or ""))
    else:
        print(msg)


def warn(msg):
    say("! " + msg, "bold yellow")


def ok(msg):
    say(msg, "bold green")


def rule(title):
    if HAVE_RICH:
        console.rule("[bold]" + title)
    else:
        print("\n=== %s ===" % title)


def table(title, columns, rows, caption=None):
    """columns: [(заголовок, 'left'|'right'|'center')]; ячейка - строка или (строка, стиль rich)."""
    if HAVE_RICH:
        t = Table(title=title, caption=caption, header_style="bold cyan")
        for head, just in columns:
            t.add_column(head, justify=just, no_wrap=True)
        for r in rows:
            t.add_row(*[Text(_plain(c), style=c[1]) if isinstance(c, tuple) else Text(str(c)) for c in r])
        console.print(t)
    else:
        print("\n" + title)
        cells = [[h for h, _ in columns]] + [[_plain(c) for c in r] for r in rows]
        w = [max(len(row[i]) for row in cells) for i in range(len(columns))]
        for k, row in enumerate(cells):
            print("  ".join(c.rjust(w[i]) if columns[i][1] != "left" else c.ljust(w[i]) for i, c in enumerate(row)))
            if k == 0:
                print("  ".join("-" * x for x in w))
        if caption:
            print(caption)


def panel(body, title, style="cyan"):
    """Рамка с текстом; удобна для команд, которые надо скопировать."""
    if HAVE_RICH:
        console.print(Panel(Text(body), title=title, title_align="left", border_style=style, expand=False))
    else:
        print("\n%s:\n%s" % (title, "\n".join("  " + ln for ln in body.splitlines())))


def good_bad(value, limit, fmt="%.0f"):
    """Ячейка со значением: зелёная в пределах порога, красная за ним."""
    return (fmt % value, "green" if value <= limit else "bold red")
