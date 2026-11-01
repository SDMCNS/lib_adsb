import requests

url = "http://127.0.0.1:8000/api/v1/flights"

payload = {
    "origin_icao": "EIDW",
    "destination_icao": "EGLL",
    "aircraft_type": "A320",
    "callsign": "EIN123",
    "cruise_alt_ft": 27000,
    "squawk": "4521",
    "icao_hex": "4CA123"
}

response = requests.post(url, json=payload)

print("Status:", response.status_code)
print("Response:", response.text)