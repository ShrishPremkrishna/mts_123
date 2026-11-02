import importlib.util,pathlib,unittest
p=pathlib.Path(__file__).with_name('summarize.py');spec=importlib.util.spec_from_file_location('summary',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class Tests(unittest.TestCase):
 def test_exact_matched_bias_difference(self):
  rows=[{'monotonic_end_s':1.,'parse_ok':True,'reports':[{'id':2,'gyro_sensor_rad_s':[.9,-1.8,.2]},{'id':7,'gyro_sensor_rad_s':[1.,-2.,.3],'estimated_bias_rad_s':[.1,-.2,.1]}]}]
  r=m.compare(rows);self.assertTrue(r['comparison_available']);self.assertEqual(r['comparisons'][0]['matched_07_reports'],1)
  self.assertTrue(all(abs(x)<1e-12 for x in r['comparisons'][0]['mean_uncal_minus_bias_minus_cal_rad_s']))
 def test_missing_or_stale_is_not_zero_error(self):
  r=m.compare([{'monotonic_end_s':1.,'parse_ok':True,'reports':[{'id':7,'gyro_sensor_rad_s':[1,2,3],'estimated_bias_rad_s':[0,0,0]}]}]);self.assertFalse(r['comparison_available']);self.assertIsNone(r['comparisons'][0]['rms_residual_rad_s'])
 def test_disjoint_streams_have_no_comparison(self):
  rows=[{'monotonic_end_s':1.,'parse_ok':True,'reports':[{'id':2,'gyro_sensor_rad_s':[0,0,0]}]},{'monotonic_end_s':2.,'parse_ok':True,'reports':[{'id':7,'gyro_sensor_rad_s':[1,2,3],'estimated_bias_rad_s':[0,0,0]}]}]
  r=m.compare(rows);self.assertTrue(r['streams_available']);self.assertFalse(r['comparison_available']);self.assertIsNone(r['comparisons'][1]['rms_residual_rad_s'])
 def test_malformed_batch_is_not_partial_measurement(self):
  r=m.compare([{'monotonic_end_s':1.,'parse_ok':False,'reports':[{'id':2,'gyro_sensor_rad_s':[1,2,3]}]}]);self.assertEqual(r['report_counts']['2'],0)
if __name__=='__main__':unittest.main()
