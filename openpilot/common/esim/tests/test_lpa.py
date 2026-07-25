from openpilot.common.esim.lpa import (
  TAG_INSTALL_RESULT_DATA,
  TAG_PROFILE_INSTALL_RESULT,
  _parse_install_result,
  _parse_sima_response,
  encode_tlv,
)


def test_parse_sima_response() -> None:
  # EUICCResponse: invalid-request-format at profile element 12, then installation aborted.
  sima_response = bytes.fromhex("300CA008300680010581010C8100")
  assert _parse_sima_response(sima_response) == {
    "peStatuses": [{
      "status": 5,
      "statusName": "invalid-request-format",
      "identification": 12,
    }],
    "profileInstallationAborted": True,
    "simaResponse": sima_response.hex().upper(),
  }


def test_parse_install_result_with_sima_diagnostic() -> None:
  sima_response = bytes.fromhex("300CA008300680010581010C8100")
  error_result = encode_tlv(0xA1, b"".join((
    encode_tlv(0x80, b"\x05"),
    encode_tlv(0x81, b"\x0c"),
    encode_tlv(0x04, sima_response),
  )))
  response = encode_tlv(
    TAG_PROFILE_INSTALL_RESULT,
    encode_tlv(TAG_INSTALL_RESULT_DATA, encode_tlv(0xA2, error_result)),
  )

  result = _parse_install_result(response)
  assert result is not None
  assert result["success"] is False
  assert result["bppCommandId"] == 5
  assert result["errorReason"] == 12
  assert result["peStatuses"] == [{
    "status": 5,
    "statusName": "invalid-request-format",
    "identification": 12,
  }]
  assert result["profileInstallationAborted"] is True
  assert result["simaResponse"] == sima_response.hex().upper()
