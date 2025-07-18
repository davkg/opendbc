from typing import get_args
from opendbc.car.body.values import CAR as BODY
from opendbc.car.chrysler.values import CAR as CHRYSLER
from opendbc.car.fca_giorgio.values import CAR as FCA_GIORGIO
from opendbc.car.ford.values import CAR as FORD
from opendbc.car.gm.values import CAR as GM
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.hyundai.values import CAR as HYUNDAI
from opendbc.car.mazda.values import CAR as MAZDA
from opendbc.car.mock.values import CAR as MOCK
from opendbc.car.nissan.values import CAR as NISSAN
from opendbc.car.subaru.values import CAR as SUBARU
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.volkswagen.values import CAR as VOLKSWAGEN

Platform = BODY | CHRYSLER | FCA_GIORGIO | FORD | GM | HONDA | HYUNDAI | MAZDA | MOCK | NISSAN | SUBARU | TOYOTA | VOLKSWAGEN
BRANDS = get_args(Platform)

PLATFORMS: dict[str, Platform] = {str(platform): platform for brand in BRANDS for platform in brand}
