import os
from twilio.rest import Client
import smtplib

account_sid = 'AC6935cc0c6fab67bd18dc9c23edb6f705'
auth_token = 'afc76eae4e07874cf2d3a2195265f965'

print(auth_token)

alert = 2

test = True

#input prams for fall detection into if statement---->

if True:
    client = Client(account_sid, auth_token)
    message = client.messages \
        .create(
        body="We have detected a fall. Please check on your loved one.",
        from_='+15075788201',
        to='+12037154454'
    )
print(auth_token)