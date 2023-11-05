import requests
import json
import pandas as pd
import csv


response_API = requests.get('https://blynk.cloud/external/api/data/get?token=nN0y_X1zx3HqPtNpBT1YdTewgf8aqg5n&period=DAY&granularityType=MINUTE&sourceType=AVG&tzName=Asia/Kolkata&format=ISO_SIMPLE&sendEvents=true&output=JSON&dataStreamId=2')
data = response_API.text
parse_json=json.loads(data)

leng= len(parse_json['data'])
df={}
df['BPM']=[]
df['DATE']=[]
df['TIME']=[]
df['SPO2']=[]

for i in range(0,leng):
    ind= leng-i-1
    df['BPM'].append(parse_json['data'][ind]['value'])
    
for i in range(0,leng):
    ind= leng-i-1
    df['DATE'].append(parse_json['data'][ind]['ts'].split(' ')[0])
    df['TIME'].append(parse_json['data'][ind]['ts'].split(' ')[1])

response_API = requests.get('https://blynk.cloud/external/api/data/get?token=nN0y_X1zx3HqPtNpBT1YdTewgf8aqg5n&period=DAY&granularityType=MINUTE&sourceType=AVG&tzName=Asia/Kolkata&format=ISO_SIMPLE&sendEvents=true&output=JSON&dataStreamId=1')
data = response_API.text
parse_json=json.loads(data)

for i in range(0,leng):
    ind= leng-i-1
    df['SPO2'].append(parse_json['data'][ind]['value'])
field_names= ['DATE', 'TIME', 'BPM', 'SPO2']

df2=[]
for i in range(0, leng):
    df3={}
    for word in field_names:
        df3[word]= df[word][i]
    df2.append(df3)

with open('Data.csv', 'w') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=field_names)
    writer.writeheader()
    writer.writerows(df2)

