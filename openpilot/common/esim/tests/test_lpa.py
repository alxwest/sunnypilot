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
    encode_tlv(0x82, sima_response),
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


def test_parse_roamless_install_result() -> None:
  # Captured from the comma eUICC: invalid-request-format at profile element 2.
  response = bytes.fromhex(
    "BF3781ABBF27658010A6CE3CAF474893ABED4A53873CE20CCCBF2F2780010B810207800C12"
    "636F6E73756D65722E7273702E776F726C645A0A98234002002024204854060D2B06010401"
    "839C270101010115A218A11680010581010C820E300CA008300680010581010281005F3740"
    "F5612B47FF1E7F060CD97DEA1D11191E29EFBC479B84C34097A9BB723D7441F4756EB96EAE"
    "A80888F54F61052E8439BC1C97CC61BCAA6AE2751CA7D90A8863BB"
  )

  result = _parse_install_result(response)
  assert result is not None
  assert result["bppCommandId"] == 5
  assert result["errorReason"] == 12
  assert result["peStatuses"] == [{
    "status": 5,
    "statusName": "invalid-request-format",
    "identification": 2,
  }]
  assert result["profileInstallationAborted"] is True
