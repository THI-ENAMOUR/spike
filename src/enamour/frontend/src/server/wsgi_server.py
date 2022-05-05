from gunicorn.app.base import BaseApplication


class WSGIServer(BaseApplication):
    def init(self, opts, args, parser=None):
        if len(args) > 1 and len(args) % 2 == 1:
            i = 1
            while i < len(args):
                key = remove_prefix(args[i], "--")
                opts[key] = args[i + 1]
                i = i + 2
        self.options = opts
        self.load_config()

    def __init__(self, flask_app, options=None):
        self.options = options or {}
        self.application = flask_app
        super().__init__()

    def load_config(self):
        config = {key: value for key, value in self.options.items() if key in self.cfg.settings and value is not None}
        for key, value in config.items():
            self.cfg.set(key.lower(), value)

    def load(self):
        return self.application


def remove_prefix(text, prefix):
    if text.startswith(prefix):
        return text[len(prefix) :]
    return text
