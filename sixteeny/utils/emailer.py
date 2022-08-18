import email
import httplib2
import os
import oauth2client
from oauth2client import client, tools, file
import base64
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from apiclient import errors, discovery
import json
import mimetypes


class Emailer(object):
    def __init__(self, config="sixteeny/utils/email_config.json", recipients=None):

        self.SCOPES = "https://www.googleapis.com/auth/gmail.send"
        self.APPLICATION_NAME = "Gmail API Python Send Email"

        # load config file
        with open(config) as f:
            config = json.load(f)

        self.CLIENT_SECRET_FILE = config["CLIENT_SECRET_FILE"]
        self.FROM = config["SENDER_ACCOUNT"]
        if type(recipients) == list:
            self.TO = recipients
        else:
            self.TO = config["RECIPIENT_ACCOUNTS"]

    def get_credentials(self):
        home_dir = os.path.expanduser("~")
        credential_dir = os.path.join(home_dir, ".credentials")
        if not os.path.exists(credential_dir):
            os.makedirs(credential_dir)
        credential_path = os.path.join(credential_dir, "gmail-python-email-send.json")
        store = oauth2client.file.Storage(credential_path)
        credentials = store.get()
        if not credentials or credentials.invalid:
            flow = client.flow_from_clientsecrets(self.CLIENT_SECRET_FILE, self.SCOPES)
            flow.user_agent = self.APPLICATION_NAME
            credentials = tools.run_flow(flow, store)
            print("Storing credentials to " + credential_path)
        return credentials

    def SendMessage(self, subject, msgHtml, msgPlain):
        credentials = self.get_credentials()
        http = credentials.authorize(httplib2.Http())
        service = discovery.build("gmail", "v1", http=http)
        for receiver in self.TO:
            message = self.CreateMessageHtml(self.FROM, receiver, subject, msgHtml, msgPlain)
            message = self.SendMessageInternal(service, "me", message)
            print(message)

    def SendMessageInternal(self, service, user_id, message):
        try:
            message = service.users().messages().send(userId=user_id, body=message).execute()
            print("Message Id: %s" % message["id"])
            return message
        except errors.HttpError as error:
            print("An error occurred: %s" % error)
            return "Error"
        return "OK"

    def CreateMessageHtml(self, sender, to, subject, msgHtml, msgPlain):
        msg = MIMEMultipart("alternative")
        msg["Subject"] = subject
        msg["From"] = sender
        msg["To"] = to
        msg.attach(MIMEText(msgPlain, "plain"))
        msg.attach(MIMEText(msgHtml, "html"))
        return {"raw": base64.urlsafe_b64encode(msg.as_bytes()).decode()}


if __name__ == "__main__":
    emailer = Emailer()
    emailer.SendMessage(
        "Test", "This is a test email.", "This is a test email.",
    )
