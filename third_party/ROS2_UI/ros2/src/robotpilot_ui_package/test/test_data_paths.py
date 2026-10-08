from robotpilot_ui_package.data_paths import data_directory, setting


def test_existing_data_directory_remains_available(monkeypatch, tmp_path):
    monkeypatch.setenv("HOME", str(tmp_path))
    legacy = tmp_path / ".openamr_ui"
    legacy.mkdir()
    assert data_directory() == str(legacy)

    current = tmp_path / ".robotpilot_ui"
    current.mkdir()
    assert data_directory() == str(current)


def test_new_environment_name_takes_precedence(monkeypatch):
    monkeypatch.setenv("OPENAMR_AUTH_DB", "/legacy/auth.sqlite3")
    monkeypatch.setenv("ROBOTPILOT_AUTH_DB", "/current/auth.sqlite3")
    assert setting("AUTH_DB", "unused") == "/current/auth.sqlite3"
