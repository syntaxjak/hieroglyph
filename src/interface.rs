use super::*;
use crossterm::event::{KeyEvent, MouseButton, MouseEventKind};
use ratatui::layout::{Alignment, Constraint, Layout, Margin, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span, Text};
use ratatui::widgets::{
    Block, BorderType, Clear, Gauge, List, ListItem, ListState, Paragraph, Wrap,
};
use ratatui::Frame;

const PRESETS: &[&str] = &["1 MiB", "10 MiB", "100 MiB"];
const ACCENT: Color = Color::Cyan;

fn card(title: impl Into<Line<'static>>, active: bool) -> Block<'static> {
    Block::bordered()
        .border_type(BorderType::Rounded)
        .border_style(Style::default().fg(if active { ACCENT } else { Color::DarkGray }))
        .title(title)
}

fn wordmark() -> Vec<Line<'static>> {
    let letters = [
        [5, 5, 7, 5, 5],
        [7, 2, 2, 2, 7],
        [7, 4, 6, 4, 7],
        [6, 5, 6, 5, 5],
        [7, 5, 5, 5, 7],
        [7, 4, 5, 5, 7],
        [4, 4, 4, 4, 7],
        [5, 5, 2, 2, 2],
        [6, 5, 6, 4, 4],
        [5, 5, 7, 5, 5],
    ];
    (0..5)
        .map(|row| {
            let mut text = String::new();
            for letter in &letters {
                for bit in (0..3).rev() {
                    text.push(if letter[row] & (1 << bit) != 0 {
                        '█'
                    } else {
                        ' '
                    });
                }
                text.push(' ');
            }
            Line::styled(text, Style::default().fg(ACCENT))
        })
        .collect()
}

pub(super) fn menu_layout(area: Rect) -> Vec<Rect> {
    let show_wordmark = area.height >= 24 && area.width >= 44;
    Layout::vertical([
        Constraint::Length(if show_wordmark { 7 } else { 3 }),
        Constraint::Min(3),
        Constraint::Length(if show_wordmark { 4 } else { 5 }),
        Constraint::Length(2),
    ])
    .split(area)
    .to_vec()
}

pub(super) fn menu_offset(selected: usize, area: Rect) -> usize {
    selected.saturating_sub(area.height.saturating_sub(2).max(1) as usize - 1)
}

pub(super) fn draw_menu(f: &mut Frame, selected: usize) {
    let parts = menu_layout(f.area());
    let mut brand = if parts[0].height >= 7 {
        wordmark()
    } else {
        vec![Line::styled(
            "HIEROGLYPH",
            Style::default().fg(ACCENT).add_modifier(Modifier::BOLD),
        )]
    };
    brand.push(Line::from("One-Time Pad workspace"));
    brand.push(Line::styled(
        "LOCAL  /  OFFLINE  /  YOUR KEYS",
        Style::default().fg(Color::DarkGray),
    ));
    f.render_widget(Paragraph::new(brand).alignment(Alignment::Center), parts[0]);

    let items: Vec<_> = WIZARD_ACTIONS
        .iter()
        .map(|a| ListItem::new(action_label(*a)))
        .collect();
    let mut state = ListState::default()
        .with_selected(Some(selected))
        .with_offset(menu_offset(selected, parts[1]));
    f.render_stateful_widget(
        List::new(items)
            .block(card(" Choose an action ", true))
            .highlight_symbol("  › ")
            .highlight_style(
                Style::default()
                    .fg(Color::Black)
                    .bg(ACCENT)
                    .add_modifier(Modifier::BOLD),
            ),
        parts[1],
        &mut state,
    );
    f.render_widget(
        Paragraph::new(action_description(WIZARD_ACTIONS[selected]))
            .wrap(Wrap { trim: true })
            .block(card(" How it works ", false)),
        parts[2],
    );
    f.render_widget(
        Paragraph::new("Click an action  ·  ↑/↓ then Enter  ·  q Quit")
            .alignment(Alignment::Center)
            .style(Style::default().fg(Color::Gray)),
        parts[3],
    );
}

fn primary_label(action: WizardAction) -> &'static str {
    match action {
        WizardAction::GeneratePad => "Create pad",
        WizardAction::QuickEncrypt | WizardAction::PadEncrypt | WizardAction::PadMessageEncrypt => {
            "Encrypt"
        }
        WizardAction::QuickDecrypt | WizardAction::PadDecrypt | WizardAction::PadMessageDecrypt => {
            "Decrypt"
        }
        WizardAction::PadBalance => "Check balance",
        WizardAction::Quit => "Quit",
    }
}

fn browsable(view: &ActionView, index: usize) -> bool {
    view.action != WizardAction::GeneratePad && view.fields.get(index).is_some_and(|f| !f.multiline)
}

struct ActionLayout {
    header: Rect,
    fields: Vec<Rect>,
    browse: Vec<Option<Rect>>,
    presets: Vec<Rect>,
    hint: Rect,
    result: Rect,
    primary: Rect,
    back: Rect,
    footer: Rect,
}

fn action_layout(area: Rect, view: &ActionView) -> ActionLayout {
    let width = area.width.min(116);
    let content = Rect::new(
        area.x + (area.width - width) / 2,
        area.y,
        width,
        area.height,
    )
    .inner(Margin {
        horizontal: 1,
        vertical: 1,
    });
    let rows = Layout::vertical([
        Constraint::Length(3),
        Constraint::Min(8),
        Constraint::Length(3),
        Constraint::Length(2),
    ])
    .split(content);
    let form_height: u16 = view
        .fields
        .iter()
        .map(|f| if f.multiline { 6 } else { 3 })
        .sum::<u16>()
        + if view.action == WizardAction::GeneratePad {
            3
        } else {
            0
        }
        + 1;
    let (form, result) = if area.width >= 96 {
        let cols = Layout::horizontal([
            Constraint::Percentage(57),
            Constraint::Length(2),
            Constraint::Min(24),
        ])
        .split(rows[1]);
        (cols[0], cols[2])
    } else {
        let stack =
            Layout::vertical([Constraint::Length(form_height), Constraint::Min(3)]).split(rows[1]);
        (stack[0], stack[1])
    };
    let mut constraints = Vec::new();
    for (i, field) in view.fields.iter().enumerate() {
        constraints.push(Constraint::Length(if field.multiline { 6 } else { 3 }));
        if i == 0 && view.action == WizardAction::GeneratePad {
            constraints.push(Constraint::Length(3));
        }
    }
    constraints.push(Constraint::Min(1));
    let slots = Layout::vertical(constraints).split(form);
    let mut fields = Vec::new();
    let mut browse = Vec::new();
    let mut presets = Vec::new();
    let mut slot = 0;
    for i in 0..view.fields.len() {
        if browsable(view, i) {
            let cols = Layout::horizontal([Constraint::Min(10), Constraint::Length(12)])
                .split(slots[slot]);
            fields.push(cols[0]);
            browse.push(Some(cols[1]));
        } else {
            fields.push(slots[slot]);
            browse.push(None);
        }
        slot += 1;
        if i == 0 && view.action == WizardAction::GeneratePad {
            presets = Layout::horizontal([Constraint::Ratio(1, 3); 3])
                .split(slots[slot])
                .to_vec();
            slot += 1;
        }
    }
    let buttons = Layout::horizontal([
        Constraint::Length(21),
        Constraint::Length(2),
        Constraint::Length(14),
        Constraint::Min(0),
    ])
    .split(rows[2]);
    ActionLayout {
        header: rows[0],
        fields,
        browse,
        presets,
        hint: slots[slot],
        result,
        primary: buttons[0],
        back: buttons[2],
        footer: rows[3],
    }
}

fn button(f: &mut Frame, area: Rect, label: &str, focused: bool, primary: bool, disabled: bool) {
    let style = if disabled {
        Style::default().fg(Color::DarkGray)
    } else if focused {
        Style::default()
            .fg(Color::Black)
            .bg(Color::LightCyan)
            .add_modifier(Modifier::BOLD)
    } else if primary {
        Style::default()
            .fg(Color::Black)
            .bg(ACCENT)
            .add_modifier(Modifier::BOLD)
    } else {
        Style::default().fg(Color::Gray)
    };
    f.render_widget(
        Paragraph::new(label)
            .alignment(Alignment::Center)
            .style(style)
            .block(card("", focused && !disabled)),
        area,
    );
}

fn cursor_position(field: &InputField) -> usize {
    if field.cursor <= field.value.len() && field.value.is_char_boundary(field.cursor) {
        field.cursor
    } else {
        field.value.len()
    }
}

fn byte_at_position(text: &str, row: usize, column: usize) -> usize {
    let mut offset = 0;
    for (i, line) in text.split('\n').enumerate() {
        if i == row {
            let mut width = 0;
            for (byte, ch) in line.char_indices() {
                width += Span::raw(ch.to_string()).width();
                if width > column {
                    return offset + byte;
                }
            }
            return offset + line.len();
        }
        offset += line.len() + 1;
    }
    text.len()
}

fn field_viewport(field: &InputField, inside: Rect) -> (usize, usize, u16, u16) {
    let before = &field.value[..cursor_position(field)];
    let row = before.bytes().filter(|b| *b == b'\n').count();
    let column = Span::raw(before.rsplit('\n').next().unwrap_or("")).width();
    let scroll_y = row
        .saturating_sub(inside.height.saturating_sub(1) as usize)
        .min(u16::MAX as usize) as u16;
    let scroll_x = column
        .saturating_sub(inside.width.saturating_sub(1) as usize)
        .min(u16::MAX as usize) as u16;
    (row, column, scroll_y, scroll_x)
}

fn draw_field(f: &mut Frame, field: &InputField, area: Rect, focused: bool) {
    let block = card(format!(" {}: ", field.label), focused);
    let inside = block.inner(area);
    let (row, column, scroll_y, scroll_x) = field_viewport(field, inside);
    let style = if focused && field.select_all {
        Style::default().fg(Color::Black).bg(ACCENT)
    } else {
        Style::default().fg(Color::White)
    };
    let text = if field.value.is_empty() {
        let placeholder = if field.multiline {
            "Type or paste your message…"
        } else if field.label == "Size" {
            "e.g. 64 KiB or 10 MiB"
        } else if field.label == "Save as" {
            "e.g. outgoing.pad"
        } else {
            "Type a path or choose Browse…"
        };
        Text::styled(placeholder, Style::default().fg(Color::DarkGray))
    } else {
        Text::styled(field.value.as_str(), style)
    };
    f.render_widget(
        Paragraph::new(text)
            .block(block)
            .scroll((scroll_y, scroll_x)),
        area,
    );
    if focused && inside.width > 0 && inside.height > 0 {
        f.set_cursor_position((
            inside.x
                + column
                    .saturating_sub(scroll_x as usize)
                    .min(inside.width as usize - 1) as u16,
            inside.y
                + row
                    .saturating_sub(scroll_y as usize)
                    .min(inside.height as usize - 1) as u16,
        ));
    }
}

fn result_text(view: &ActionView) -> String {
    if view.status_kind == StatusKind::Ready {
        return format!(
            "{}\n\nFill in the details, then choose {}.\n\n{}",
            action_label(view.action),
            primary_label(view.action),
            action_description(view.action)
        );
    }
    let mut text = view.status.join("\n\n");
    if let Some((title, output)) = &view.output_panel {
        text.push_str(&format!("\n\n{title}\n{output}"));
    }
    text
}

fn result_widget(view: &ActionView) -> Paragraph<'static> {
    let (title, color) = match view.status_kind {
        StatusKind::Ready => (" Before you start ", Color::Gray),
        StatusKind::Running => (" Creating your pad… ", ACCENT),
        StatusKind::Success => (" Done ✓ ", Color::Green),
        StatusKind::Error => (" Needs attention ", Color::LightRed),
    };
    Paragraph::new(result_text(view))
        .wrap(Wrap { trim: false })
        .block(card(title, view.focus == Focus::Output).border_style(Style::default().fg(color)))
}

pub(super) fn draw_action(f: &mut Frame, view: &ActionView) {
    if f.area().width < 60 || f.area().height < 24 {
        f.render_widget(Paragraph::new("HIEROGLYPH\n\nPlease resize to at least 60 columns × 24 rows.\nEsc: back to workspace")
            .wrap(Wrap { trim: true }), f.area());
        return;
    }
    let layout = action_layout(f.area(), view);
    f.render_widget(
        Paragraph::new(vec![
            Line::styled(
                format!("HIEROGLYPH  /  {}", action_label(view.action)),
                Style::default().fg(ACCENT).add_modifier(Modifier::BOLD),
            ),
            Line::from("Your files. Your pad. Everything stays local."),
        ]),
        layout.header,
    );
    for (i, field) in view.fields.iter().enumerate() {
        draw_field(
            f,
            field,
            layout.fields[i],
            view.focus == Focus::Fields && view.selected == i && !view.busy,
        );
        if let Some(area) = layout.browse[i] {
            button(
                f,
                area,
                "Browse…",
                view.focus == Focus::Browse && view.selected == i,
                false,
                view.busy,
            );
        }
    }
    for (i, area) in layout.presets.iter().enumerate() {
        let selected =
            parse_pad_size(&view.fields[0].value).ok() == parse_pad_size(PRESETS[i]).ok();
        button(
            f,
            *area,
            &format!("{} {}", if selected { "●" } else { "○" }, PRESETS[i]),
            view.focus == Focus::Presets && view.preset_idx == i,
            selected,
            view.busy,
        );
    }
    let hint = if view.action == WizardAction::GeneratePad {
        "Custom sizes: 64 KiB, 5 MiB. Pads are never replaced."
    } else {
        "Type directly into a field, or click Browse to choose a file."
    };
    f.render_widget(
        Paragraph::new(hint)
            .style(Style::default().fg(Color::Gray))
            .wrap(Wrap { trim: true }),
        layout.hint,
    );
    if let Some(progress) = &view.pad_progress {
        let parts =
            Layout::vertical([Constraint::Length(3), Constraint::Min(0)]).split(layout.result);
        f.render_widget(
            Gauge::default()
                .block(card(" Creating your pad… ", true))
                .gauge_style(Style::default().fg(ACCENT))
                .ratio(progress.ratio())
                .label(format!(
                    "{} / {}",
                    format_bytes(progress.done),
                    format_bytes(progress.target_len)
                )),
            parts[0],
        );
        f.render_widget(
            Paragraph::new(format!(
                "Saving to {}\n\nPlease wait. Secret pad bytes are never displayed.",
                progress.path
            ))
            .wrap(Wrap { trim: true }),
            parts[1],
        );
    } else {
        f.render_widget(
            result_widget(view).scroll((
                view.output_scroll
                    .min(result_scroll_limit(view, layout.result)),
                0,
            )),
            layout.result,
        );
    }
    button(
        f,
        layout.primary,
        if view.busy {
            "Creating…"
        } else {
            primary_label(view.action)
        },
        view.focus == Focus::Primary,
        true,
        view.busy,
    );
    button(
        f,
        layout.back,
        "← Back",
        view.focus == Focus::Back,
        false,
        view.busy,
    );
    let footer = if view.busy {
        "Please wait for generation to finish."
    } else if view.focus == Focus::Output {
        "↑/↓ or PgUp/PgDn: scroll results  ·  Tab: next control  ·  Esc: back"
    } else {
        "Tab: next  ·  Enter: next / activate  ·  Ctrl+A: select all\nF2: browse  ·  F5: run  ·  PgUp/PgDn: results  ·  Esc: back"
    };
    f.render_widget(
        Paragraph::new(footer).style(Style::default().fg(Color::Gray)),
        layout.footer,
    );
    if view.focus == Focus::Candidates {
        draw_picker(f, view);
    }
}

fn set_value(field: &mut InputField, value: String) {
    field.cursor = value.len();
    field.value = value;
    field.select_all = false;
}

fn insert_text(field: &mut InputField, text: &str) {
    let text: String = text
        .replace("\r\n", "\n")
        .chars()
        .filter(|c| !c.is_control() || (*c == '\n' && field.multiline))
        .collect();
    if text.is_empty() {
        return;
    }
    if field.select_all {
        set_value(field, String::new());
    }
    field.cursor = cursor_position(field);
    field.value.insert_str(field.cursor, &text);
    field.cursor += text.len();
}

pub(super) fn handle_char_input(field: &mut InputField, key: KeyCode, modifiers: KeyModifiers) {
    field.cursor = cursor_position(field);
    if modifiers.contains(KeyModifiers::CONTROL) {
        match key {
            KeyCode::Char('a') => field.select_all = true,
            KeyCode::Char('u') => set_value(field, String::new()),
            KeyCode::Home => {
                field.cursor = 0;
                field.select_all = false;
            }
            KeyCode::End => {
                field.cursor = field.value.len();
                field.select_all = false;
            }
            _ => {}
        }
        return;
    }
    let previous = field.value[..field.cursor]
        .char_indices()
        .last()
        .map(|(i, _)| i)
        .unwrap_or(0);
    let next = field.value[field.cursor..]
        .chars()
        .next()
        .map(|c| field.cursor + c.len_utf8())
        .unwrap_or(field.cursor);
    match key {
        KeyCode::Char(c) if !modifiers.contains(KeyModifiers::ALT) => {
            insert_text(field, &c.to_string())
        }
        KeyCode::Enter if field.multiline => insert_text(field, "\n"),
        KeyCode::Backspace | KeyCode::Delete if field.select_all => set_value(field, String::new()),
        KeyCode::Backspace => {
            field.value.replace_range(previous..field.cursor, "");
            field.cursor = previous;
        }
        KeyCode::Delete => {
            field.value.replace_range(field.cursor..next, "");
        }
        KeyCode::Left => {
            field.cursor = if field.select_all { 0 } else { previous };
            field.select_all = false;
        }
        KeyCode::Right => {
            field.cursor = if field.select_all {
                field.value.len()
            } else {
                next
            };
            field.select_all = false;
        }
        KeyCode::Home => {
            field.cursor = field.value[..field.cursor]
                .rfind('\n')
                .map(|i| i + 1)
                .unwrap_or(0);
            field.select_all = false;
        }
        KeyCode::End => {
            field.cursor += field.value[field.cursor..]
                .find('\n')
                .unwrap_or(field.value.len() - field.cursor);
            field.select_all = false;
        }
        KeyCode::Up | KeyCode::Down if field.multiline => {
            let before = &field.value[..field.cursor];
            let row = before.bytes().filter(|b| *b == b'\n').count();
            let column = Span::raw(before.rsplit('\n').next().unwrap_or("")).width();
            let target = if key == KeyCode::Up {
                row.saturating_sub(1)
            } else {
                (row + 1).min(field.value.bytes().filter(|b| *b == b'\n').count())
            };
            field.cursor = byte_at_position(&field.value, target, column);
            field.select_all = false;
        }
        _ => {}
    }
}

fn controls(view: &ActionView) -> Vec<(Focus, usize)> {
    let mut controls = Vec::new();
    for i in 0..view.fields.len() {
        controls.push((Focus::Fields, i));
        if browsable(view, i) {
            controls.push((Focus::Browse, i));
        }
        if i == 0 && view.action == WizardAction::GeneratePad {
            for p in 0..PRESETS.len() {
                controls.push((Focus::Presets, p));
            }
        }
    }
    controls.extend([(Focus::Primary, 0), (Focus::Back, 0), (Focus::Output, 0)]);
    controls
}

fn move_focus(view: &mut ActionView, backwards: bool) {
    let controls = controls(view);
    let index = if view.focus == Focus::Presets {
        view.preset_idx
    } else if matches!(view.focus, Focus::Fields | Focus::Browse) {
        view.selected
    } else {
        0
    };
    let position = controls
        .iter()
        .position(|item| *item == (view.focus, index))
        .unwrap_or(0);
    let (focus, index) =
        controls[(position + if backwards { controls.len() - 1 } else { 1 }) % controls.len()];
    view.focus = focus;
    if focus == Focus::Presets {
        view.preset_idx = index;
    }
    if matches!(focus, Focus::Fields | Focus::Browse) {
        view.selected = index;
    }
    if focus == Focus::Fields {
        let field = &mut view.fields[index];
        field.cursor = field.value.len();
        field.select_all = !field.multiline && !field.value.is_empty();
    }
}

fn result_scroll_limit(view: &ActionView, rect: Rect) -> u16 {
    result_widget(view)
        .line_count(rect.width.saturating_sub(2))
        .saturating_sub(rect.height as usize)
        .min(u16::MAX as usize) as u16
}

fn scroll_results(view: &mut ActionView, area: Rect, down: bool, amount: u16) {
    let maximum = result_scroll_limit(view, action_layout(area, view).result);
    view.output_scroll = view.output_scroll.min(maximum);
    view.output_scroll = if down {
        view.output_scroll.saturating_add(amount).min(maximum)
    } else {
        view.output_scroll.saturating_sub(amount)
    };
}

pub(super) fn handle_action_event(view: &mut ActionView, event: &Event, area: Rect) -> bool {
    if view.busy {
        return false;
    }
    if let Event::Key(key) = event {
        if key.kind == KeyEventKind::Release {
            return false;
        }
        if key.code == KeyCode::Esc {
            if view.focus == Focus::Candidates {
                view.focus = Focus::Fields;
                return false;
            }
            return true;
        }
    }
    if area.width < 60 || area.height < 24 {
        return false;
    }
    if view.focus == Focus::Candidates {
        handle_picker_event(view, event, area);
        return false;
    }
    match event {
        Event::Paste(text) if view.focus == Focus::Fields => {
            insert_text(&mut view.fields[view.selected], text)
        }
        Event::Key(key) => match key.code {
            KeyCode::Tab => move_focus(view, false),
            KeyCode::BackTab => move_focus(view, true),
            KeyCode::F(5) if key.kind == KeyEventKind::Press => run_action_now(view),
            KeyCode::F(2) if browsable(view, view.selected) => open_picker(view, view.selected),
            KeyCode::PageDown => scroll_results(view, area, true, 5),
            KeyCode::PageUp => scroll_results(view, area, false, 5),
            KeyCode::Enter if view.focus == Focus::Fields => {
                if view.fields[view.selected].multiline {
                    handle_char_input(&mut view.fields[view.selected], key.code, key.modifiers);
                } else if view.selected + 1 < view.fields.len() {
                    view.selected += 1;
                    view.fields[view.selected].select_all = true;
                } else {
                    view.focus = Focus::Primary;
                }
            }
            KeyCode::Enter | KeyCode::Char(' ') if view.focus != Focus::Fields => {
                match view.focus {
                    Focus::Primary if key.kind == KeyEventKind::Press => run_action_now(view),
                    Focus::Back => return true,
                    Focus::Browse => open_picker(view, view.selected),
                    Focus::Presets => {
                        set_value(&mut view.fields[0], PRESETS[view.preset_idx].to_string())
                    }
                    _ => {}
                }
            }
            KeyCode::Up | KeyCode::Down if view.focus == Focus::Output => {
                scroll_results(view, area, key.code == KeyCode::Down, 1)
            }
            KeyCode::Left | KeyCode::Right if view.focus == Focus::Presets => {
                view.preset_idx = (view.preset_idx
                    + if key.code == KeyCode::Left {
                        PRESETS.len() - 1
                    } else {
                        1
                    })
                    % PRESETS.len();
            }
            _ if view.focus == Focus::Fields => {
                handle_char_input(&mut view.fields[view.selected], key.code, key.modifiers)
            }
            KeyCode::Up => move_focus(view, true),
            KeyCode::Down => move_focus(view, false),
            _ => {}
        },
        Event::Mouse(mouse) => {
            let layout = action_layout(area, view);
            let point = (mouse.column, mouse.row).into();
            if matches!(
                mouse.kind,
                MouseEventKind::ScrollDown | MouseEventKind::ScrollUp
            ) && layout.result.contains(point)
            {
                scroll_results(view, area, mouse.kind == MouseEventKind::ScrollDown, 3);
            }
            if mouse.kind != MouseEventKind::Down(MouseButton::Left) {
                return false;
            }
            if layout.back.contains(point) {
                return true;
            }
            if layout.primary.contains(point) {
                view.focus = Focus::Primary;
                run_action_now(view);
            } else if layout.result.contains(point) {
                view.focus = Focus::Output;
            } else {
                for (i, rect) in layout.fields.iter().enumerate() {
                    if rect.contains(point) {
                        view.focus = Focus::Fields;
                        view.selected = i;
                        let inside = rect.inner(Margin {
                            horizontal: 1,
                            vertical: 1,
                        });
                        let (_, _, scroll_y, scroll_x) = field_viewport(&view.fields[i], inside);
                        view.fields[i].cursor = byte_at_position(
                            &view.fields[i].value,
                            mouse.row.saturating_sub(inside.y) as usize + scroll_y as usize,
                            mouse.column.saturating_sub(inside.x) as usize + scroll_x as usize,
                        );
                        view.fields[i].select_all = false;
                    }
                    if layout.browse[i].is_some_and(|rect| rect.contains(point)) {
                        open_picker(view, i);
                        return false;
                    }
                }
                for (i, rect) in layout.presets.iter().enumerate() {
                    if rect.contains(point) {
                        view.focus = Focus::Presets;
                        view.preset_idx = i;
                        set_value(&mut view.fields[0], PRESETS[i].to_string());
                    }
                }
            }
        }
        _ => {}
    }
    false
}

fn open_picker(view: &mut ActionView, field: usize) {
    view.browse_field = field;
    view.selected = field;
    let path = Path::new(view.fields[field].value.trim());
    let directory = if path.is_dir() {
        path
    } else {
        path.parent().unwrap_or(Path::new("."))
    };
    view.picker_dir = std::fs::canonicalize(directory)
        .unwrap_or_else(|_| env::current_dir().unwrap_or_else(|_| PathBuf::from(".")));
    view.focus = Focus::Candidates;
    refresh_picker(view);
}

fn refresh_picker(view: &mut ActionView) {
    view.available.clear();
    view.candidate_idx = 0;
    view.picker_error = None;
    match std::fs::read_dir(&view.picker_dir)
        .and_then(|entries| entries.collect::<io::Result<Vec<_>>>())
    {
        Ok(entries) => {
            let mut paths: Vec<_> = entries
                .into_iter()
                .map(|e| e.path())
                .filter(|p| p.is_dir() || p.is_file())
                .collect();
            paths.sort_by_key(|p| (!p.is_dir(), p.clone()));
            if let Some(parent) = view.picker_dir.parent() {
                view.available.push(parent.to_string_lossy().into_owned());
            }
            view.available
                .extend(paths.into_iter().map(|p| p.to_string_lossy().into_owned()));
        }
        Err(error) => {
            view.picker_error = Some(format!(
                "Cannot read this folder: {error}. Press Esc and type a path instead."
            ))
        }
    }
}

fn choose_file(view: &mut ActionView) {
    let Some(path) = view.available.get(view.candidate_idx).cloned() else {
        return;
    };
    if Path::new(&path).is_dir() {
        view.picker_dir = PathBuf::from(path);
        refresh_picker(view);
    } else {
        set_value(&mut view.fields[view.browse_field], path.clone());
        if view.action == WizardAction::QuickDecrypt && view.browse_field == 0 {
            set_value(
                &mut view.fields[1],
                auto_glyph_key_path(&path).unwrap_or_default(),
            );
        }
        view.focus = Focus::Fields;
    }
}

fn picker_layout(area: Rect) -> Vec<Rect> {
    let rect = area.inner(Margin {
        horizontal: 3,
        vertical: 2,
    });
    Layout::vertical([
        Constraint::Length(3),
        Constraint::Min(3),
        Constraint::Length(3),
    ])
    .split(rect)
    .to_vec()
}

fn draw_picker(f: &mut Frame, view: &ActionView) {
    let parts = picker_layout(f.area());
    f.render_widget(
        Clear,
        f.area().inner(Margin {
            horizontal: 3,
            vertical: 2,
        }),
    );
    f.render_widget(
        Paragraph::new(view.picker_dir.to_string_lossy().into_owned()).block(card(
            format!(
                " Choose {} ",
                view.fields[view.browse_field].label.to_lowercase()
            ),
            true,
        )),
        parts[0],
    );
    if let Some(error) = &view.picker_error {
        f.render_widget(
            Paragraph::new(error.as_str()).wrap(Wrap { trim: true }),
            parts[1],
        );
    } else {
        let items: Vec<_> = view
            .available
            .iter()
            .map(|path| {
                let path = Path::new(path);
                let name = if Some(path) == view.picker_dir.parent() {
                    ".. (parent folder)".into()
                } else {
                    path.file_name().unwrap_or_default().to_string_lossy()
                };
                ListItem::new(format!(
                    "{} {}",
                    if path.is_dir() {
                        "[folder]"
                    } else {
                        "[file]  "
                    },
                    name
                ))
            })
            .collect();
        let mut state = ListState::default()
            .with_selected(Some(view.candidate_idx))
            .with_offset(menu_offset(view.candidate_idx, parts[1]));
        f.render_stateful_widget(
            List::new(items)
                .block(card(" Files and folders ", true))
                .highlight_style(Style::default().fg(Color::Black).bg(ACCENT))
                .highlight_symbol("› "),
            parts[1],
            &mut state,
        );
    }
    button(f, parts[2], "← Cancel  ·  Esc", false, false, false);
}

fn handle_picker_event(view: &mut ActionView, event: &Event, area: Rect) {
    let parts = picker_layout(area);
    match event {
        Event::Key(KeyEvent {
            code: KeyCode::Enter,
            kind: KeyEventKind::Press,
            ..
        }) => choose_file(view),
        Event::Key(KeyEvent {
            code: KeyCode::Up, ..
        }) => view.candidate_idx = view.candidate_idx.saturating_sub(1),
        Event::Key(KeyEvent {
            code: KeyCode::Down,
            ..
        }) => {
            view.candidate_idx =
                (view.candidate_idx + 1).min(view.available.len().saturating_sub(1))
        }
        Event::Key(KeyEvent {
            code: KeyCode::Backspace,
            ..
        }) => {
            if let Some(parent) = view.picker_dir.parent() {
                view.picker_dir = parent.to_path_buf();
                refresh_picker(view);
            }
        }
        Event::Mouse(mouse) => {
            let point = (mouse.column, mouse.row).into();
            if mouse.kind == MouseEventKind::ScrollDown {
                view.candidate_idx =
                    (view.candidate_idx + 1).min(view.available.len().saturating_sub(1));
            }
            if mouse.kind == MouseEventKind::ScrollUp {
                view.candidate_idx = view.candidate_idx.saturating_sub(1);
            }
            if mouse.kind == MouseEventKind::Down(MouseButton::Left) {
                if parts[2].contains(point) {
                    view.focus = Focus::Fields;
                }
                let inside = parts[1].inner(Margin {
                    horizontal: 1,
                    vertical: 1,
                });
                if inside.contains(point) && view.picker_error.is_none() {
                    let index =
                        menu_offset(view.candidate_idx, parts[1]) + (mouse.row - inside.y) as usize;
                    if index < view.available.len() {
                        view.candidate_idx = index;
                        choose_file(view);
                    }
                }
            }
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::TestDir;
    use crossterm::event::MouseEvent;
    use ratatui::{backend::TestBackend, Terminal};

    fn key(code: KeyCode) -> Event {
        Event::Key(KeyEvent::new(code, KeyModifiers::NONE))
    }

    fn click(rect: Rect) -> Event {
        Event::Mouse(MouseEvent {
            kind: MouseEventKind::Down(MouseButton::Left),
            column: rect.x + 1,
            row: rect.y + 1,
            modifiers: KeyModifiers::NONE,
        })
    }

    fn render(view: &ActionView, width: u16, height: u16) -> String {
        let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
        terminal.draw(|f| draw_action(f, view)).unwrap();
        terminal
            .backend()
            .buffer()
            .content
            .chunks(width as usize)
            .map(|row| row.iter().map(|cell| cell.symbol()).collect::<String>())
            .collect::<Vec<_>>()
            .join("\n")
    }

    #[test]
    fn create_pad_form_has_named_controls_at_desktop_and_compact_sizes() {
        let view = build_action_view(WizardAction::GeneratePad).unwrap();
        for (width, height) in [(116, 32), (96, 24), (80, 24), (60, 24)] {
            let screen = render(&view, width, height);
            for label in [
                "HIEROGLYPH",
                "Size:",
                "Save as:",
                "1 MiB",
                "10 MiB",
                "100 MiB",
                "Create pad",
                "Back",
            ] {
                assert!(
                    screen.contains(label),
                    "Missing {label} at {width}x{height}:\n{screen}"
                );
            }
            let layout = action_layout(Rect::new(0, 0, width, height), &view);
            assert!(layout.result.height >= 3);
            assert!(layout.fields.iter().all(|rect| rect.height >= 3));
            assert!(!screen.contains("e: edit"));
        }
    }

    #[test]
    fn workspace_has_a_pixel_wordmark_and_a_compact_fallback() {
        for (width, height) in [(80, 24), (116, 32), (55, 16)] {
            let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
            terminal.draw(|f| draw_menu(f, 0)).unwrap();
            let text: String = terminal
                .backend()
                .buffer()
                .content
                .iter()
                .map(|cell| cell.symbol())
                .collect();
            assert!(text.contains("One-Time Pad workspace"));
            if height >= 24 {
                assert!(text.contains('█'));
                assert!(menu_layout(Rect::new(0, 0, width, height))[1].height >= 11);
            } else {
                assert!(text.contains("HIEROGLYPH"));
            }
        }
    }

    #[test]
    fn mouse_click_positions_the_caret_instead_of_appending() {
        let area = Rect::new(0, 0, 100, 30);
        let mut view = build_action_view(WizardAction::GeneratePad).unwrap();
        let field = action_layout(area, &view).fields[1];
        let mut event = click(field);
        if let Event::Mouse(mouse) = &mut event {
            mouse.column += 3;
        }
        handle_action_event(&mut view, &event, area);
        handle_action_event(&mut view, &key(KeyCode::Char('2')), area);
        assert_eq!(view.fields[1].value, "pad2.bin");
    }

    #[test]
    fn fields_edit_immediately_and_enter_never_submits_while_in_a_field() {
        let area = Rect::new(0, 0, 100, 30);
        let mut view = build_action_view(WizardAction::GeneratePad).unwrap();
        handle_action_event(&mut view, &Event::Paste("64 KiB".into()), area);
        assert_eq!(view.fields[0].value, "64 KiB");
        handle_action_event(&mut view, &key(KeyCode::Enter), area);
        assert_eq!(view.selected, 1);
        handle_action_event(&mut view, &Event::Paste("outgoing.pad".into()), area);
        handle_action_event(&mut view, &key(KeyCode::Enter), area);
        assert_eq!(view.focus, Focus::Primary);
        assert_eq!(view.status_kind, StatusKind::Ready);
        assert!(!view.busy);
        assert_eq!(view.fields[1].value, "outgoing.pad");
    }

    #[test]
    fn tab_reaches_every_button_and_shift_tab_reverses_it() {
        for action in WIZARD_ACTIONS
            .iter()
            .copied()
            .filter(|a| *a != WizardAction::Quit)
        {
            let mut view = build_action_view(action).unwrap();
            let expected = controls(&view);
            for item in expected.iter().cycle().skip(1).take(expected.len()) {
                move_focus(&mut view, false);
                assert_eq!(view.focus, item.0);
                if item.0 == Focus::Presets {
                    assert_eq!(view.preset_idx, item.1);
                }
                if matches!(item.0, Focus::Fields | Focus::Browse) {
                    assert_eq!(view.selected, item.1);
                }
            }
            move_focus(&mut view, true);
            assert_eq!(view.focus, Focus::Output);
        }
    }

    #[test]
    fn mouse_presets_and_keyboard_presets_match() {
        let area = Rect::new(0, 0, 100, 30);
        let mut view = build_action_view(WizardAction::GeneratePad).unwrap();
        let layout = action_layout(area, &view);
        handle_action_event(&mut view, &click(layout.presets[1]), area);
        assert_eq!(view.fields[0].value, "10 MiB");
        handle_action_event(&mut view, &key(KeyCode::Right), area);
        handle_action_event(&mut view, &key(KeyCode::Enter), area);
        assert_eq!(view.fields[0].value, "100 MiB");
        assert!(handle_action_event(&mut view, &click(layout.back), area));
        assert!(!view.busy);
    }

    #[test]
    fn unicode_editing_selection_paste_and_multiline_caret() {
        let mut field = InputField::new("Message", "a界é\nlast".into(), true);
        handle_char_input(&mut field, KeyCode::Home, KeyModifiers::NONE);
        handle_char_input(&mut field, KeyCode::Up, KeyModifiers::NONE);
        assert_eq!(field.cursor, 0);
        handle_char_input(&mut field, KeyCode::Right, KeyModifiers::NONE);
        handle_char_input(&mut field, KeyCode::Delete, KeyModifiers::NONE);
        assert_eq!(field.value, "aé\nlast");
        insert_text(&mut field, "🙂");
        assert_eq!(field.value, "a🙂é\nlast");
        handle_char_input(&mut field, KeyCode::Backspace, KeyModifiers::NONE);
        assert_eq!(field.value, "aé\nlast");
        handle_char_input(&mut field, KeyCode::Char('a'), KeyModifiers::CONTROL);
        insert_text(&mut field, "first\r\nsecond\u{1b}\u{7}");
        assert_eq!(field.value, "first\nsecond");
        assert!(!field.select_all);
        assert_eq!(byte_at_position("a界b", 0, 2), 1);
        assert_eq!(byte_at_position("a界b", 0, 3), 4);
    }

    #[test]
    fn browser_navigates_folders_selects_keys_and_cancels_without_changes() {
        let dir = TestDir::new();
        let nested = dir.path("nested");
        std::fs::create_dir(&nested).unwrap();
        std::fs::write(dir.path("nested/message.glyph"), b"ciphertext").unwrap();
        std::fs::write(dir.path("nested/message.glyphkey.bin"), b"pad").unwrap();
        let mut view = build_action_view(WizardAction::QuickDecrypt).unwrap();
        set_value(&mut view.fields[0], dir.path("missing.glyph"));
        open_picker(&mut view, 0);
        view.candidate_idx = view.available.iter().position(|p| p == &nested).unwrap();
        choose_file(&mut view);
        assert_eq!(view.picker_dir, PathBuf::from(&nested));
        let encrypted = dir.path("nested/message.glyph");
        view.candidate_idx = view.available.iter().position(|p| p == &encrypted).unwrap();
        choose_file(&mut view);
        assert_eq!(view.focus, Focus::Fields);
        assert_eq!(view.fields[0].value, encrypted);
        assert_eq!(
            view.fields[1].value,
            dir.path("nested/message.glyphkey.bin")
        );
        open_picker(&mut view, 1);
        assert!(!handle_action_event(
            &mut view,
            &key(KeyCode::Esc),
            Rect::new(0, 0, 80, 24)
        ));
        assert_eq!(
            view.fields[1].value,
            dir.path("nested/message.glyphkey.bin")
        );
        assert!(handle_action_event(
            &mut view,
            &key(KeyCode::Esc),
            Rect::new(0, 0, 80, 24)
        ));
    }

    #[test]
    fn browser_reports_missing_folders_and_handles_empty_lists() {
        let dir = TestDir::new();
        let mut view = build_action_view(WizardAction::PadBalance).unwrap();
        view.focus = Focus::Candidates;
        view.picker_dir = PathBuf::from(dir.path("missing"));
        refresh_picker(&mut view);
        assert!(view.picker_error.as_ref().unwrap().contains("Cannot read"));
        choose_file(&mut view);
        assert!(render(&view, 80, 24).contains("Cannot read"));
        assert_eq!(view.fields[0].value, DEFAULT_PAD_PATH);
    }

    #[test]
    fn primary_click_validates_and_busy_controls_cannot_change_pad() {
        let dir = TestDir::new();
        let area = Rect::new(0, 0, 80, 24);
        let mut view = build_action_view(WizardAction::GeneratePad).unwrap();
        let layout = action_layout(area, &view);
        set_value(&mut view.fields[0], "nonsense".into());
        set_value(&mut view.fields[1], dir.path("new.pad"));
        handle_action_event(&mut view, &click(layout.primary), area);
        assert_eq!(view.status_kind, StatusKind::Error);
        assert!(render(&view, 80, 24).contains("Needs attention"));
        assert!(!Path::new(&view.fields[1].value).exists());
        set_value(&mut view.fields[0], "32 B".into());
        handle_action_event(&mut view, &click(layout.primary), area);
        assert_eq!(view.status_kind, StatusKind::Running);
        assert!(!handle_action_event(&mut view, &click(layout.back), area));
        handle_action_event(&mut view, &click(layout.presets[2]), area);
        handle_action_event(&mut view, &Event::Paste("overwrite".into()), area);
        assert_eq!(view.fields[0].value, "32 B");
        let progress = view.pad_progress.as_mut().unwrap();
        progress.step().unwrap();
        assert!(progress.finished);
        assert_eq!(std::fs::metadata(dir.path("new.pad")).unwrap().len(), 32);
    }

    #[test]
    fn results_scroll_to_the_end_and_resize_without_becoming_blank() {
        let area = Rect::new(0, 0, 80, 24);
        let mut view = build_action_view(WizardAction::PadMessageDecrypt).unwrap();
        view.status_kind = StatusKind::Success;
        view.output_panel = Some((
            "Message".into(),
            (0..50).map(|n| format!("line {n}\n")).collect(),
        ));
        for _ in 0..100 {
            scroll_results(&mut view, area, true, 5);
        }
        assert!(render(&view, 80, 24).contains("line 49"));
        assert!(render(&view, 116, 40).contains("line 49"));
        for _ in 0..100 {
            scroll_results(&mut view, area, false, 5);
        }
        assert_eq!(view.output_scroll, 0);
    }

    #[test]
    fn small_terminals_do_not_activate_invisible_controls() {
        let mut view = build_action_view(WizardAction::GeneratePad).unwrap();
        let area = Rect::new(0, 0, 40, 10);
        handle_action_event(&mut view, &key(KeyCode::F(5)), area);
        assert!(!view.busy);
        assert!(render(&view, 40, 10).contains("Please resize"));
        assert!(handle_action_event(&mut view, &key(KeyCode::Esc), area));
    }
}
