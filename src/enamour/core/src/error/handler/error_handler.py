"""
Base class erstellen "error_handler" (Siehe controller class):
    besitzt eine abstract method "handle_error(error: BaseException)"

Spezifische class erstellen "ActionOrganizerErrorHandler" welche von ErrorHandler erbt (sihe pose controller):
    der action_organizer und die action_queue wird im constructor übergeben
    besitzt die spezifische methode "handle_error(error: BaseException)
        in der handle_error methode ganze viele if/elif mit is_instance(error, HIER_ERROR_CLASS_EINSETZEN)
        behandlung des spezifischen fehlers, wie es sinn macht (evtl. noch leer)

Hinzufügen eins try and expect in action_organizer um die execute_action methode

Hinzufügen von spezielleren Klassen wie validation_error


"""
import abc


class ErrorHandler(BaseException):
    @abc.abstractmethod
    def handle(self, error: BaseException, **kwargs):
        raise NotImplementedError
