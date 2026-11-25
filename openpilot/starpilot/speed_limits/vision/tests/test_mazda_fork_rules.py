"""filter-dev sign rules ported to StarPilot's reader: SCHOOL plaque, implausible reads, newest strong read."""
from collections import deque
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

from openpilot.starpilot.speed_limits.vision import model, producer
from openpilot.starpilot.speed_limits.vision.model import Detection, HistoryEntry, VisionModelCore

MPH = 0.44704


def bgr(hue, sat, val, shape):
  import cv2
  hsv = np.zeros((*shape, 3), np.uint8)
  hsv[:] = (hue, sat, val)
  return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def school_sign(h=90, w=60):
  crop = bgr(0, 0, 230, (h, w))  # white body
  crop[:int(h * .3)] = bgr(28, 220, 230, (int(h * .3), w))  # solid yellow SCHOOL band
  return crop


def plain_sign(h=90, w=60):
  return bgr(0, 0, 230, (h, w))


def core(now=1000.):
  c = VisionModelCore.__new__(VisionModelCore)
  c.is_metric = False
  c.history = deque()
  c.published_speed_limit_mph = 0
  c.published_confidence = 0.
  c.previous_published_speed_limit_mph = 0
  c.last_publish_change_at = 0.
  c.last_detection_at = 0.
  c.last_published_support_at = 0.
  c.episode = 0
  c.published_support_count = 0
  c.school_zone_read = (0, 0.)
  c._observe_now = None
  return c


class SchoolPlaqueTests(unittest.TestCase):
  def test_band_over_pale_body_is_school(self):
    self.assertTrue(VisionModelCore._is_school_zone_sign(school_sign()))

  def test_plain_regulatory_sign_is_not(self):
    self.assertFalse(VisionModelCore._is_school_zone_sign(plain_sign()))

  def test_all_yellow_advisory_is_not(self):  # a yellow body is an advisory/warning sign, not a plaque over a limit
    self.assertFalse(VisionModelCore._is_school_zone_sign(bgr(28, 220, 230, (90, 60))))

  def test_degenerate_crops(self):
    self.assertFalse(VisionModelCore._is_school_zone_sign(np.zeros((1, 1, 3), np.uint8)))
    self.assertFalse(VisionModelCore._is_school_zone_sign(np.zeros((10, 10), np.uint8)))


class SchoolObserveTests(unittest.TestCase):
  def run_reads(self, c, reads):
    out = []
    for now, det in reads:
      with patch.object(c, 'infer', return_value=det):
        r = c.observe(np.zeros((4, 4, 3), np.uint8), now=now)
      out.append(r[0] if r else 0)
    return out

  def test_school_value_is_not_published_for_10_s_and_other_values_are(self):
    c = core()
    c.school_zone_read = (20, 1010.)
    self.assertEqual(self.run_reads(c, [(1000., Detection(20, .95)), (1000.2, Detection(20, .95))]), [0, 0])
    # blocked school reads still count in the 2 s history (filter-dev the same): the real 25 publishes once it outnumbers them
    self.assertEqual(self.run_reads(c, [(1000.4, Detection(25, .95)), (1000.6, Detection(25, .95)), (1000.8, Detection(25, .95))]),
                     [0, 0, 25])
    c = core()
    c.school_zone_read = (20, 1010.)
    self.assertEqual(self.run_reads(c, [(1011., Detection(20, .95)), (1011.2, Detection(20, .95))]), [20, 20])

  def test_plaque_read_withdraws_a_just_published_school_value(self):
    c = core()
    c.published_speed_limit_mph, c.previous_published_speed_limit_mph, c.last_publish_change_at = 20, 30, 999.
    c.classifier_net = object()
    c._observe_now = 1000.
    with patch.object(model, '_primary_regulatory_panel', side_effect=lambda x: x), \
         patch.object(c, '_square_resize', return_value=np.zeros((128, 128, 3), np.uint8)), \
         patch('cv2.dnn.blobFromImage', return_value=None):
      c.classifier_net = NS(setInput=lambda b: None, forward=lambda *a: np.array([[0.] * 3 + [10.] + [0.] * 14]))
      c.classifier_logits_layer = None
      c.classifier_input_size = 128
      c.last_classifier_forward_count, c.last_classifier_forward_duration_s = 0, 0.
      self.assertIsNone(c._classify_speed_limit_from_model(school_sign()))  # the classifier reads 20 (index 3)
    self.assertEqual(c.school_zone_read, (20, 1010.))
    self.assertEqual(c.published_speed_limit_mph, 0)
    self.assertEqual(c.previous_published_speed_limit_mph, 20)


class ImplausibleTests(unittest.TestCase):
  def observe(self, c, now, det, other):
    with patch.object(c, 'infer', return_value=det):
      r = c.observe(np.zeros((4, 4, 3), np.uint8), now=now, other_limit_ms=other)
    return r[0] if r else 0

  def test_20_over_map_and_car_is_ignored_old_sign_stays(self):
    c = core()
    for t in (1000., 1000.2):
      self.observe(c, t, Detection(25, .95), 25 * MPH)
    self.assertEqual(self.observe(c, 1001., Detection(45, .95), 25 * MPH), 25)  # route 0e 905 s, Costco lot
    self.assertEqual(self.observe(c, 1001.2, Detection(45, .95), 25 * MPH), 25)
    self.assertNotIn(45, [e.speed_limit_mph for e in c.history])

  def test_19_over_passes(self):
    c = core()
    self.assertEqual([self.observe(c, t, Detection(44, .95), 25 * MPH) for t in (1000., 1000.2)][-1], 44)

  def test_alone_50_and_over_is_ignored_and_clears_the_old_sign(self):
    c = core()
    for t in (1000., 1000.2):
      self.observe(c, t, Detection(25, .95), None)
    self.assertEqual(self.observe(c, 1001., Detection(60, .95), 0.), 0)  # route 05 118 s
    c = core()
    self.assertEqual([self.observe(c, t, Detection(45, .95), 0.) for t in (1000., 1000.2)][-1], 45)

  def test_no_sources_no_filter(self):
    c = core()
    self.assertEqual([self.observe(c, t, Detection(65, .95), None) for t in (1000., 1000.2)][-1], 65)


class NewestReadTests(unittest.TestCase):
  def test_strong_newest_read_beats_weak_older_reads(self):
    # filter-dev route 12 t=262: a far 30 read twice at 0.64, the new road's 25 once with strong consensus at 0.95
    c = core()
    c.published_speed_limit_mph = 35
    c.history = deque([HistoryEntry(30, .64, 1.), HistoryEntry(30, .64, 1.1), HistoryEntry(25, .95, 1.2, True)])
    self.assertEqual(c._confirm_detection(), (25, .95))

  def test_a_strong_other_value_keeps_the_vote(self):
    c = core()
    c.published_speed_limit_mph = 35
    c.history = deque([HistoryEntry(30, .9, 1.), HistoryEntry(30, .9, 1.1), HistoryEntry(25, .95, 1.2, True)])
    self.assertNotEqual(c._confirm_detection(), (25, .95))

  def test_low_speed_bar_still_applies(self):
    c = core()
    c.published_speed_limit_mph = 35
    c.history = deque([HistoryEntry(30, .64, 1.), HistoryEntry(25, .85, 1.2, True)])
    self.assertNotEqual(c._confirm_detection(), (25, .85))


class ProducerOtherLimitTests(unittest.TestCase):
  def sm(self, now, map_mps=None, dash=None, map_age=0, dash_age=0):
    data, seen, valid, mono = {}, {}, {}, {}
    if map_mps is not None:
      data['mapdOut'] = NS(speedLimit=map_mps); seen['mapdOut'] = valid['mapdOut'] = True; mono['mapdOut'] = now - map_age
    if dash is not None:
      data['slcDashboardObservation'] = dash; seen['slcDashboardObservation'] = valid['slcDashboardObservation'] = True
      mono['slcDashboardObservation'] = now - dash_age
    return NS(seen=seen, valid=valid, logMonoTime=mono, data=data)

  def other(self, sm, now):
    class SM:
      def __init__(s): s.seen, s.valid, s.logMonoTime = sm.seen, sm.valid, sm.logMonoTime
      def __getitem__(s, k): return sm.data[k]
    return producer._other_limit_ms(SM(), now)

  def test_sources(self):
    now = 10**12
    dash = NS(status='valid', speedMps=25 * MPH, validUntilMonoTime=now + 1)
    self.assertIsNone(self.other(self.sm(now), now))
    self.assertAlmostEqual(self.other(self.sm(now, 30 * MPH, dash), now), 30 * MPH, places=4)
    self.assertAlmostEqual(self.other(self.sm(now, 0., dash), now), 25 * MPH, places=4)
    self.assertEqual(self.other(self.sm(now, 0.), now), 0.)
    self.assertIsNone(self.other(self.sm(now, 30 * MPH, map_age=300_000_000), now))  # stale map, no dash
    expired = NS(status='valid', speedMps=25 * MPH, validUntilMonoTime=now - 1)
    self.assertEqual(self.other(self.sm(now, None, expired), now), 0.)
    self.assertEqual(self.other(self.sm(now, None, NS(status='unknown', speedMps=0., validUntilMonoTime=0)), now), 0.)


if __name__ == '__main__':
  unittest.main()
