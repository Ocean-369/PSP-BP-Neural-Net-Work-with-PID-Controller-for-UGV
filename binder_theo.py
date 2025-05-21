class BinderPage:
    def __init__(self, controller):
        self.controller = controller

class BinderSpace:
    def __init__(self):
        self.pages = []

    def add_page(self, controller):
        self.pages.append(BinderPage(controller))